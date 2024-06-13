#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

class Waypoint : public rclcpp::Node
{

public:

	Waypoint() : Node("waypoint")
	{
	auto home = std::getenv("HOME");
	auto timestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	auto gm_time = std::gmtime(&timestamp);
	char buffer[100]; std::strftime(buffer, 100, "/sim_ws/src/waypoint/logs/wp-%Y-%m-%d-%H-%M-%S.csv",gm_time);
       	file_path_ = std::string(home) + buffer; file_.open(file_path_);
       	if (!file_.is_open()) {
	       	RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str()); rclcpp::shutdown();
       	}
       	else { 
		RCLCPP_INFO(this->get_logger(), "Saving waypoints...");
       	}
       	subscription_ = this->create_subscription<nav_msgs::msg::Odometry>( "pf/pose/odom", 10, std::bind(&Waypoint::save_waypoint, this, _1));
} 
~Waypoint() 
{ 
	if (file_.is_open()) 
	{ file_.close();
	       	RCLCPP_INFO(this->get_logger(), "Goodbye"); 
	} 
} 
private: 
void save_waypoint(const nav_msgs::msg::Odometry::SharedPtr msg) 
{ tf2::Quaternion quaternion( 
		msg->pose.pose.orientation.x,
	       	msg->pose.pose.orientation.y,
	       	msg->pose.pose.orientation.z,
	       	msg->pose.pose.orientation.w); 
	tf2::Matrix3x3 m(quaternion); 
	double roll, pitch, yaw; 
	m.getRPY(roll, pitch, yaw);

	double speed = std::sqrt( 
			std::pow(msg->twist.twist.linear.x, 2) + 
			std::pow(msg->twist.twist.linear.y, 2) + 
			std::pow(msg->twist.twist.linear.z, 2)); 
	if (msg->twist.twist.linear.x > 0.) 
	{ 
		RCLCPP_INFO(this->get_logger(), "Linear X: %f", msg->twist.twist.linear.x); 
	} 
	file_ << msg->pose.pose.position.x << ", "
	      << msg->pose.pose.position.y << ", "
	      << yaw << ", " 
	      << speed << "\n";
} 
	std::string file_path_; 
	std::ofstream file_; 
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_; 
}; 
int main(int argc, char *argv[]) { 
	rclcpp::init(argc, argv); 
	rclcpp::spin(std::make_shared<Waypoint>()); 
	rclcpp::shutdown(); 
	return 0;
}
