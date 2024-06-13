#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <functional>
using namespace std::placeholders;
class Safety : public rclcpp::Node {
// The class that handles emergency braking
public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages
	
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */


        /// TODO: create ROS subscribers and publishers DONE
        ls_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Safety::scan_callback, this,_1));
	odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",10,std::bind(&Safety::odom_callback, this,_1));
	drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDrive>("drive",10);
 }
private:
           
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr ls_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDrive>::SharedPtr drive_publisher_;


	  
	    float cur_speed = 0.0;
	    /// TODO: create ROS subscribers and publishers DONE
	   

	    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
	    {
		/// TODO: update current speed
	
		cur_speed = msg->twist.twist.linear.x;
		//RCLCPP_INFO(this->get_logger(), "whats up? current speed is '%f'",cur_speed);	
	    }

	    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
	    {
		/// TODO: calculate TTC
		float ttc = 0.0;
		std::vector<float> v_angles;
		std::vector<float> ranges = scan_msg->ranges;
	float current_angle = scan_msg->angle_min;
	while (current_angle <= scan_msg->angle_max){
		v_angles.push_back(std::cos(current_angle) * cur_speed);
		current_angle += scan_msg->angle_increment;

	}
	RCLCPP_INFO(this->get_logger(),"scan message angle min:'%f'",scan_msg->angle_min);
	RCLCPP_INFO (this->get_logger(), "scan message angle max: '%f'", scan_msg->angle_max);
	RCLCPP_INFO (this->get_logger(), "scan message angle increment: '%f'",scan_msg->angle_increment);

	RCLCPP_INFO(this->get_logger(), "range: '%f'",ranges[0]);
	for (long unsigned int j = 0; j<v_angles.size(); j++){
		ttc = -(ranges[j]/v_angles[j]);
		if (std::isnan(ttc) || std::isinf(ttc) || std::isinf(-ttc)){
			continue;
		}
		current_angle += scan_msg->angle_increment;
		if (ttc <= 10 || ttc>= -10){
        /// TODO: publish drive/brake message
	auto message = std::make_unique<ackermann_msgs::msg::AckermannDrive>();
	//RCLCPP_INFO(this->get_logger(),"tcc: '%f'",ttc);
	message->speed = 0.0;
	drive_publisher_->publish(std::move(message));
	//RCLCPP_INFO(this->get_logger(),"tcc: '%f' braking?",ttc);
	       }
	}

    }

  
};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

