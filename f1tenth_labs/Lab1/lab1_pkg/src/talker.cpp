#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    Talker() : Node("talker")
    {
        this->declare_parameter<double>("v", 0.0);
        this->declare_parameter<double>("d", 0.0);

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Talker::publish_message, this));
    }

private:
    void publish_message()
    {
        this->get_parameter("v", v_);
        this->get_parameter("d", d_);

        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed = v_;
        message.drive.steering_angle = d_;
        publisher_->publish(message);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double v_;
    double d_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
