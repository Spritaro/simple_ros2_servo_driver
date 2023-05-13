#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

#include "serial_servo_msgs/msg/serial_servo.hpp"
#include "serial_servo_msgs/msg/serial_servo_array.hpp"

using namespace std::chrono_literals;

class SerialServoTest : public rclcpp::Node
{
public:
    explicit SerialServoTest() : Node("serial_servo_test")
    {
        serial_servo_msgs::msg::SerialServo servo;
        msg_ = std::make_shared<serial_servo_msgs::msg::SerialServoArray>();
        msg_->device_file = "/dev/ttyS1";
        msg_->serial_servos.emplace_back(servo);
        msg_->serial_servos.emplace_back(servo);

        pub_ = create_publisher<serial_servo_msgs::msg::SerialServoArray>("/serial_servo_target", 10);
    }

    void send_new_angles()
    {
        msg_->serial_servos.at(0).id = 14;
        msg_->serial_servos.at(0).target_angle = 0.0;
        msg_->serial_servos.at(0).target_time = 1.0;

        msg_->serial_servos.at(1).id = 17;
        msg_->serial_servos.at(1).target_angle = 0.0;
        msg_->serial_servos.at(1).target_time = 1.0;

        RCLCPP_INFO(this->get_logger(), "send new angles");
        pub_->publish(*msg_);
        usleep(1 * 1000 * 1000);

        msg_->serial_servos.at(0).id = 14;
        msg_->serial_servos.at(0).target_angle = 30.0;
        msg_->serial_servos.at(0).target_time = 2.0;

        msg_->serial_servos.at(1).id = 17;
        msg_->serial_servos.at(1).target_angle = 30.0;
        msg_->serial_servos.at(1).target_time = 2.0;

        RCLCPP_INFO(this->get_logger(), "send new angles");
        pub_->publish(*msg_);
        usleep(1 * 1000 * 1000);

        msg_->serial_servos.at(0).id = 14;
        msg_->serial_servos.at(0).target_angle = 0.0;
        msg_->serial_servos.at(0).target_time = 1.0;

        msg_->serial_servos.at(1).id = 17;
        msg_->serial_servos.at(1).target_angle = 0.0;
        msg_->serial_servos.at(1).target_time = 1.0;

        RCLCPP_INFO(this->get_logger(), "send new angles");
        pub_->publish(*msg_);
        usleep(1 * 1000 * 1000);

        msg_->serial_servos.at(0).id = 14;
        msg_->serial_servos.at(0).target_angle = -10.0;
        msg_->serial_servos.at(0).target_time = 0.5;

        msg_->serial_servos.at(1).id = 17;
        msg_->serial_servos.at(1).target_angle = -10.0;
        msg_->serial_servos.at(1).target_time = 0.5;

        RCLCPP_INFO(this->get_logger(), "send new angles");
        pub_->publish(*msg_);
        usleep(1 * 1000 * 1000);

        msg_->serial_servos.at(0).id = 14;
        msg_->serial_servos.at(0).target_angle = 0.0;
        msg_->serial_servos.at(0).target_time = 0.1;

        msg_->serial_servos.at(1).id = 17;
        msg_->serial_servos.at(1).target_angle = 0.0;
        msg_->serial_servos.at(1).target_time = 0.1;

        RCLCPP_INFO(this->get_logger(), "send new angles");
        pub_->publish(*msg_);
        usleep(1 * 1000 * 1000);
    }

private:
    std::shared_ptr<serial_servo_msgs::msg::SerialServo> servo1_, servo2_;
    std::shared_ptr<serial_servo_msgs::msg::SerialServoArray> msg_;
    rclcpp::Publisher<serial_servo_msgs::msg::SerialServoArray>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SerialServoTest>();

    node->send_new_angles();

    rclcpp::shutdown();

    return 0;
}