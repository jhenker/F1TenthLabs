#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class Relay : public rclcpp::Node
{
	public:
		Relay()
		: Node("relay")
		{
		  subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>("drive",10, std::bind(&Relay::topic_callback, this, _1));
		  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("drive_relay",10);

		}

	private:
		void topic_callback( ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
			ackermann_msgs::msg::AckermannDrive new_message;
			double old_speed = msg->speed * 3;
			double old_steering_angle = msg->steering_angle*3;

			new_message.steering_angle = old_steering_angle;
			new_message.speed = old_speed;

			RCLCPP_INFO(this->get_logger(), "I heard speed:'%f' and steering_angle: '%f'", new_message.speed,new_message.steering_angle);
			publisher_->publish(new_message);
		}
		rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr subscription_;
		rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
};
	int main(int argc, char * argv[]){
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<Relay>());
		rclcpp::shutdown();
		return 0;
	}
