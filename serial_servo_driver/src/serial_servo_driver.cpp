#include "serial_servo_driver/serial_servo_driver.hpp"

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
