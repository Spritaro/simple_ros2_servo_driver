#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "serial_servo_msgs/msg/serial_servo.hpp"
#include "serial_servo_msgs/msg/serial_servo_array.hpp"

using namespace std::chrono_literals;

class ServoState
{
public:
    explicit ServoState(const uint8_t id, const float ang_acc):
        id_(id),
        ang_acc_(ang_acc),
        cur_ang_(0.0f),
        tar_ang_(0.0f),
        cur_ang_vel_(0.0f),
        cur_time_(0.0f),
        acc_time_(0.0f),
        dec_time_(0.0f),
        tar_time_(0.0f),
        tar_ccw_(1.0f)
    {
        assert(ang_acc_ > 0.0f);
    }

    void set_target_angle(const float tar_ang, const float tar_time)
    {
        tar_ang_     = tar_ang;
        tar_time_    = tar_time;
        cur_ang_vel_ = 0.0f;
        cur_time_    = 0.0f;

        float ang_diff = fabsf(tar_ang_ - cur_ang_);

        // 移動時間が足りない場合は時間を伸ばす
        if(ang_acc_ * tar_time_ * tar_time_ < 4.0f * ang_diff)
        {
            tar_time_ = 2.0f * sqrtf(ang_diff / ang_acc_);
        }

        // 加速時間・減速時間を計算
        acc_time_ = ( tar_time_ - sqrtf( tar_time_ * tar_time_ - 4.0f * ang_diff / ang_acc_) ) / 2.0f;
        dec_time_ = tar_time - acc_time_;

        // 時計回り・半時計回りを設定
        if(tar_ang_ - cur_ang_ >= 0)
        {
            tar_ccw_ = 1.0f;
        }
        else
        {
            tar_ccw_ = -1.0f;
        }
    }

    float update_current_angle(const float delta_time)
    {
        // 停止
        if(cur_time_ >= tar_time_)
        {
            cur_ang_ = tar_ang_;
        }
        // 減速
        else if(cur_time_ >= dec_time_)
        {
            cur_ang_vel_ -= tar_ccw_ * ang_acc_ * delta_time;
            cur_ang_ += cur_ang_vel_ * delta_time;
            cur_time_ += delta_time;
        }
        // 等速
        else if(cur_time_ >= acc_time_)
        {
            cur_ang_vel_ = tar_ccw_ * ang_acc_ * acc_time_;
            cur_ang_ += cur_ang_vel_ * delta_time;
            cur_time_ += delta_time;
        }
        // 加速
        else
        {
            cur_ang_vel_ += tar_ccw_ * ang_acc_ * delta_time;
            cur_ang_ += cur_ang_vel_ * delta_time;
            cur_time_ += delta_time;
        }
        return cur_ang_;
    }

    uint8_t get_id()
    {
        return id_;
    }

    uint get_new_position()
    {
        const float min_pos = 3500.0f;
        const float max_pos = 11500.0f;
        const float min_ang = -135.0f;
        const float max_ang = 135.0f;

        return (max_pos - min_pos) * (cur_ang_ - min_ang) / (max_ang - min_ang) + min_pos;
    }

private:
    uint8_t id_;
    float ang_acc_;
    float cur_ang_;
    float tar_ang_;
    float cur_ang_vel_;
    float cur_time_;    // current time
    float acc_time_;    // acceleration end time
    float dec_time_;    // deceleration begin time
    float tar_time_;    // target time
    float tar_ccw_;     // target counter-clockwise
};

class SerialServoDriver : public rclcpp::Node
{
public:

    explicit SerialServoDriver():
        Node("serial_servo_driver"),
        nb_servo_(0)
    {
        get_parameters();

        setup_device();

        // message event handler
        auto callback =
        [this](const serial_servo_msgs::msg::SerialServoArray::SharedPtr msg) -> void
        {
            if(msg->device_file != device_file_)
            {
                return;
            }

            for(auto & serial_servo : msg->serial_servos)
            {
                // add new servo if new ID was received
                if(servo_id_to_no_.find(serial_servo.id) == servo_id_to_no_.end() )
                {
                    RCLCPP_INFO(this->get_logger(), "add servo id %d", serial_servo.id);
                    add_servo(serial_servo.id);
                }
                RCLCPP_INFO(this->get_logger(), "target_angle %f", serial_servo.target_angle );
                RCLCPP_INFO(this->get_logger(), "target_time %f", serial_servo.target_time );

                servo_states_.at( servo_id_to_no_.at(serial_servo.id) ).set_target_angle(
                    serial_servo.target_angle,
                    serial_servo.target_time
                );
            }
        };
        std::string topic_name = "/serial_servo_target";
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        sub_ = this->create_subscription<serial_servo_msgs::msg::SerialServoArray>(
            topic_name, qos, callback);

        // timer event handler
        auto control_servo =
        [this]() -> void
        {
            for(auto & servo_state : servo_states_)
            {
                servo_state.update_current_angle(delta_time_);
                set_position(servo_state.get_id(), servo_state.get_new_position());
            }
        };
        timer_ = create_wall_timer(std::chrono::milliseconds(update_period_), control_servo);
    }

    void close_device()
    {
        tcflush(fd_, TCIOFLUSH);       // 未送信・未受信のデータを破棄
        close(fd_);
    }

private:

    void get_parameters()
    {
        RCLCPP_INFO(this->get_logger(), "get parameters...");

        this->declare_parameter("device_file");
        this->declare_parameter("max_angular_velocity");
        this->declare_parameter("angular_acceleration");
        this->declare_parameter("update_period");

        this->get_parameter_or("device_file", device_file_, std::string("/dev/ttyS1"));
        this->get_parameter_or("angular_acceleration", ang_acc_, 352.9f * 2.0f);  // deg/sec^2
        this->get_parameter_or("update_period", update_period_, 20u);   // millisec

        delta_time_ = static_cast<float>(update_period_) / 1000.0f; // millisec to sec
    }

    void setup_device()
    {
        RCLCPP_INFO(this->get_logger(), "setup device...");

        // open device
        fd_ = open(device_file_.c_str(), O_RDWR);
        RCLCPP_INFO(this->get_logger(), "device no %d", fd_);
        if(fd_ == -1)
        {
            RCLCPP_INFO(this->get_logger(), "device open error");
            rclcpp::shutdown();
            return;
        }

        // serial communication settings
        struct termios tio;
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);         // 特殊文字の処理を無効化
        tio.c_cflag &= ~CSIZE;   // 文字サイズ指定用のビットをクリアする
        tio.c_cflag &= ~CRTSCTS; // RTS/CTSフロー制御を無効にする
        tio.c_cflag |= CREAD;    // 受信を有効にする
        tio.c_cflag |= CLOCAL;   // モデムの制御線を無視する
        tio.c_cflag |= CS8;      // 8bit
        tio.c_cflag |= PARENB;   // パリティビット有効化
        tio.c_cflag &= ~PARODD;  // 偶数パリティ

        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME]= 0;

        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);

        tcflush(fd_, TCIOFLUSH);       // 未送信・未受信のデータを破棄
        tcsetattr(fd_, TCSANOW, &tio); // デバイスに設定を適用
    }

    void add_servo(uint8_t id)
    {
        servo_id_to_no_.emplace(id, nb_servo_);
        servo_states_.emplace_back(ServoState(id, ang_acc_));
        nb_servo_++;
    }

    uint set_position(uint8_t id, uint pos)
    {
            // send data
            uint8_t cmd = 0x80;

            uint8_t pos_h, pos_l;
            pos_h = (pos >> 7) & 0x7f;
            pos_l = (pos >> 0) & 0x7f;

            uint8_t buf[255];
            buf[0] = cmd | id;
            buf[1] = pos_h;
            buf[2] = pos_l;

            int len = 3;
            write(fd_, buf, len);
            RCLCPP_INFO(this->get_logger(), "target  %x %d", buf[0], (buf[1]<<7)+buf[2]);
            usleep(1000);

            // receive data
            len = read(fd_, buf, 3);
            RCLCPP_INFO(this->get_logger(), "current %x %d", buf[3], (buf[4]<<7)+buf[5]);

            return (buf[4]<<7)+buf[5];
    }


    int fd_;
    int nb_servo_;
    std::string device_file_;

    float ang_acc_;

    uint update_period_;
    float delta_time_;

    std::vector<ServoState> servo_states_;
    std::map<uint8_t, uint8_t> servo_id_to_no_;

    rclcpp::Subscription<serial_servo_msgs::msg::SerialServoArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};



int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // ノード生成
    auto node = std::make_shared<SerialServoDriver>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    // デバイスをクローズ
    node->close_device();

    return 0;
}
