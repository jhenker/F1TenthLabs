#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
	{

	public:
	  Talker()
	  : Node("talker_node")
	  {
		  publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("drive",10);
		  this->declare_parameter("v",3.34);
		  this->declare_parameter("d",3.78);
		  timer_ = this->create_wall_timer(
		    1000ms, std::bind(&Talker::timer_callback, this));

	  }
	void timer_callback(){
		  auto message = ackermann_msgs::msg::AckermannDrive();
		  message.speed = this->get_parameter("v").as_double();
		  message.steering_angle = this->get_parameter("d").as_double();
		 

		  RCLCPP_INFO(this->get_logger(), "Publishing: speed=%f, steering_angle=%f",message.speed, message.steering_angle);
		  publisher_->publish(message);
	  }
	  rclcpp::TimerBase::SharedPtr timer_;
	  rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr publisher_;
	};
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Talker>());
	rclcpp::shutdown();
	return 0;
}
