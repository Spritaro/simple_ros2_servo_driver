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
#include "serial_servo_msgs/msg/serial_servo_array_array.hpp"

using namespace std::chrono_literals;

class ServoState
{
public:
    explicit ServoState(const uint8_t id):
        id_(id),
        cur_ang_(0.0f),
        tar_ang_(0.0f),
        cur_ang_vel_(0.0f),
        cur_time_(0.0f),
        tar_time_(0.0f)
    {
    }

    void set_target_angle(const int device_file_id, const float tar_ang, const float tar_time)
    {
        device_file_id_ = device_file_id;

        tar_ang_     = tar_ang;
        tar_time_    = tar_time;
        cur_time_    = 0.0f;

        // 移動速度を計算
        if( fabsf(tar_time_) < 0.00001)
        {
            cur_ang_vel_ = 0.0f;
        }
        else
        {
            cur_ang_vel_ = (tar_ang_ - cur_ang_) / tar_time;
        }
    }

    float update_current_angle(const float delta_time)
    {
        // トルクOFF
        if(tar_ang_ == 1000)
        {
            return cur_ang_;
        }

        // 停止
        if(cur_time_ >= tar_time_)
        {
            cur_ang_ = tar_ang_;
        }
        // 移動
        else
        {
            cur_ang_ += cur_ang_vel_ * delta_time;
            cur_time_ += delta_time;
        }
        return cur_ang_;
    }

    int get_device_file_id()
    {
        return device_file_id_;
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

        // トルクOFF
        if (tar_ang_ == 1000)
        {
            return 0;
        }

        return (max_pos - min_pos) * (cur_ang_ - min_ang) / (max_ang - min_ang) + min_pos;
    }

private:
    int device_file_id_;
    uint8_t id_;
    float cur_ang_;
    float tar_ang_;
    float cur_ang_vel_;
    float cur_time_;    // current time
    float tar_time_;    // target time
};

class SerialServoDriver : public rclcpp::Node
{
public:

    explicit SerialServoDriver():
        Node("serial_servo_driver"),
        is_registered_(false),
        nb_servo_(0)
    {
        get_parameters();

        // message event handler
        auto callback =
        [this](const serial_servo_msgs::msg::SerialServoArrayArray::SharedPtr msg) -> void
        {
            if(!is_registered_)
            {
                is_registered_ = true;

                for(auto & serial_servo_array : msg->serial_servo_arrays)
                {
                    // add new serial port if new device file name was received
                    if( device_file_name_to_id_.find(serial_servo_array.device_file) == device_file_name_to_id_.end() )
                    {
                        // RCLCPP_INFO(this->get_logger(), "add serial port %s", serial_servo_array.device_file.c_str());
                        setup_device(serial_servo_array.device_file);
                    }

                    for(auto & serial_servo : serial_servo_array.serial_servos)
                    {
                        // add new servo if new ID was received
                        if( servo_id_to_no_.find(serial_servo.id) == servo_id_to_no_.end() )
                        {
                            // RCLCPP_INFO(this->get_logger(), "add servo id %d", serial_servo.id);
                            add_servo(serial_servo.id);
                        }
                    }
                }
            }
                
            for(auto & serial_servo_array : msg->serial_servo_arrays)
            {
                for(auto & serial_servo : serial_servo_array.serial_servos)
                {
                    // RCLCPP_INFO(this->get_logger(), "target_angle %f", serial_servo.target_angle );
                    // RCLCPP_INFO(this->get_logger(), "target_time %f", serial_servo.target_time );

                    servo_states_.at( servo_id_to_no_.at(serial_servo.id) ).set_target_angle(
                        device_file_name_to_id_.at(serial_servo_array.device_file),
                        serial_servo.target_angle,
                        serial_servo.target_time
                    );
                }
            }
        };
        std::string topic_name = "/serial_servo_target";
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        sub_ = this->create_subscription<serial_servo_msgs::msg::SerialServoArrayArray>(
            topic_name, qos, callback);

        // timer event handler
        auto control_servo =
        [this]() -> void
        {
            for(auto & servo_state : servo_states_)
            {
                servo_state.update_current_angle(delta_time_);
                set_position(
                    servo_state.get_device_file_id(),
                    servo_state.get_id(),
                    servo_state.get_new_position()
                );
            }
        };
        timer_ = create_wall_timer(std::chrono::milliseconds(update_period_), control_servo);
    }

    void close_device()
    {
        for(auto device : device_file_name_to_id_)
        {
            tcflush(device.second, TCIOFLUSH);       // 未送信・未受信のデータを破棄
            close(device.second);
        }
    }

private:

    void get_parameters()
    {
        RCLCPP_INFO(this->get_logger(), "get parameters...");
        this->declare_parameter("update_period");
        this->get_parameter_or("update_period", update_period_, 20u);   // millisec
        delta_time_ = static_cast<float>(update_period_) / 1000.0f; // millisec to sec
    }

    void setup_device(std::string device_file_name)
    {
        RCLCPP_INFO(this->get_logger(), "setup device...");

        // open device
        int device_file_id = open(device_file_name.c_str(), O_RDWR);
        RCLCPP_INFO(this->get_logger(), "device no %d", device_file_id);
        if(device_file_id == -1)
        {
            RCLCPP_INFO(this->get_logger(), "device open error");
            rclcpp::shutdown();
            return;
        }

        // serial communication settings
        struct termios tio;
        tcgetattr(device_file_id, &tio);
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

        tcflush(device_file_id, TCIOFLUSH);       // 未送信・未受信のデータを破棄
        tcsetattr(device_file_id, TCSANOW, &tio); // デバイスに設定を適用

        device_file_name_to_id_.emplace(device_file_name, device_file_id);
    }

    void add_servo(uint8_t id)
    {
        servo_id_to_no_.emplace(id, nb_servo_);
        servo_states_.emplace_back(ServoState(id));
        nb_servo_++;
    }

    uint set_position(int device_file_id, uint8_t id, uint pos)
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
            write(device_file_id, buf, len);
            // RCLCPP_INFO(this->get_logger(), "target  %x %d", buf[0], (buf[1]<<7)+buf[2]);
            usleep(800);

            // receive data
            len = read(device_file_id, buf, 3);
            // RCLCPP_INFO(this->get_logger(), "current %x %d", buf[3], (buf[4]<<7)+buf[5]);

            return (buf[4]<<7)+buf[5];
    }

    bool is_registered_;

    int nb_servo_;
    std::string device_file_;

    uint update_period_;
    float delta_time_;

    std::map<std::string, int> device_file_name_to_id_;

    std::vector<ServoState> servo_states_;
    std::map<uint8_t, uint8_t> servo_id_to_no_;

    rclcpp::Subscription<serial_servo_msgs::msg::SerialServoArrayArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
