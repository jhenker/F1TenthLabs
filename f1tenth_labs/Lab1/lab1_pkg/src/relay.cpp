#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Relay : public rclcpp::Node
{
public:
    Relay() : Node("relay")
    {
        subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 10,
            std::bind(&Relay::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

private:
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        auto new_msg = ackermann_msgs::msg::AckermannDriveStamped();
        new_msg.drive.speed = msg->drive.speed * 3.0;
        new_msg.drive.steering_angle = msg->drive.steering_angle * 3.0;
        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}
