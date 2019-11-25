#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class SerialServoDriver : public rclcpp::Node
{
public:
    int fd;

    explicit SerialServoDriver(const int &deg)
     : Node("serial_servo_driver")
    {
        // タイマー実行されるイベントハンドラ
        auto control_servo =
        [this]() -> void
        {
            if(servo_deg == 7500)
                servo_deg = 8500;
            else
                servo_deg = 7500;

            RCLCPP_INFO(this->get_logger(), "%d", servo_deg);

            set_position(1, servo_deg);
            set_position(2, servo_deg);
            set_position(3, servo_deg);
            set_position(4, servo_deg);
        };

        // タイマー
        timer_ = create_wall_timer(500ms, control_servo);

        // デバイスをオープン
        fd = open("/dev/ttyS1", O_RDWR);
        RCLCPP_INFO(this->get_logger(), "device no %d", fd);
        if(fd == -1)
        {
            RCLCPP_INFO(this->get_logger(), "device open error");
            rclcpp::shutdown();
            return;
        }

        // シリアル通信設定
        struct termios tio;
        tcgetattr(fd, &tio);
        tio.c_cflag &= ~CSIZE;   // 文字サイズ指定用のビットをクリアする
        tio.c_cflag &= ~CRTSCTS; // RTS/CTSフロー制御を無効にする
        tio.c_cflag |= CREAD;    // 受信を有効にする
        tio.c_cflag |= CLOCAL;   // モデムの制御線を無視する
        tio.c_cflag |= CS8;      // 8bit
        tio.c_cflag |= PARENB;   // パリティビット有効化
        tio.c_cflag &= ~PARODD;  // 偶数パリティ

        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_cc[VMIN] = 0;             //      ノンブロッキング
        tio.c_cc[VTIME]= 0;

        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);

        tcflush(fd, TCIOFLUSH);       // 未送信・未受信のデータを破棄
        tcsetattr(fd, TCSANOW, &tio); // デバイスに設定を適用
    }

private:
    int servo_deg;
    rclcpp::TimerBase::SharedPtr timer_;

    unsigned int set_position(unsigned char id, unsigned int pos)
    {
            // 送信
            unsigned char cmd = 0x80;
            // unsigned char id  = id;
            // unsigned int  pos = 7500;

            unsigned char pos_h, pos_l;
            pos_h = (pos >> 7) & 0x7f;
            pos_l = (pos >> 0) & 0x7f;

            unsigned char buf[255];
            buf[0] = cmd | id;
            buf[1] = pos_h;
            buf[2] = pos_l;

            int len = 3;
            write(fd, buf, len);
            usleep(10000);

            // 受信
            len = read(fd, buf, 6);
            usleep(10000);
            RCLCPP_INFO(this->get_logger(), "target  %x %d", buf[0], (buf[1]<<7)+buf[2]);
            RCLCPP_INFO(this->get_logger(), "current %x %d", buf[3], (buf[4]<<7)+buf[5]);

            return (buf[4]<<7)+buf[5];
    }
};



int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // ノード生成
    auto node = std::make_shared<SerialServoDriver>(9500);

    // 受信
    //unsigned char buf[255];
    //int len = read(node->fd, buf, sizeof(buf));
    //RCLCPP_INFO(node->get_logger(), "len = %d", len);
    //RCLCPP_INFO(node->get_logger(), "%x %x %x", buf[0], buf[1], buf[2]);
    //RCLCPP_INFO(node->get_logger(), "%x %x %x", buf[3], buf[4], buf[5]);

    rclcpp::spin(node);
    rclcpp::shutdown();

    // デバイスをクローズ
    tcflush(node->fd, TCIOFLUSH);       // 未送信・未受信のデータを破棄
    close(node->fd);

    return 0;
}
