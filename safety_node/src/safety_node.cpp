
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <vector>
//#include <subscription.hpp>
using std::placeholders::_1;


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

        /// TODO: create ROS subscribers and publishers
	laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Safety::scan_callback, this, _1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));
	ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    }

private:
    double speed = 0.0;
    double ittc_threshold = 1;
    /// TODO: create ROS subscribers and publishers
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
	//RCLCPP_INFO(this->get_logger(), "Received odometry message");
	speed = msg->twist.twist.linear.x;
	//RCLCPP_INFO(this->get_logger(), "Current speed: %f m/s", speed);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
	std::vector<double> ittc_values;
    	ittc_values.reserve(scan_msg->ranges.size());

	double angle = scan_msg->angle_min;
	for(size_t i = 0; i < scan_msg->ranges.size(); i++){
		double distance = scan_msg->ranges[i];
		if (std::isnan(distance) || std::isinf(distance) || distance <= 0.0){
			ittc_values.push_back(std::numeric_limits<double>::infinity());
		}else{
			double rel_vel = speed * std::cos(angle);
			//RCLCPP_INFO(this->get_logger(), "Distance: %f, Angle: %f, Relative Velocity: %f, SPEED: %f", distance, angle, rel_vel, speed);

      			double ittc;
      			if (rel_vel <= 0){ // Prevent division by zero and negative velocities
        			ittc = std::numeric_limits<double>::infinity();
      			}else{
        			ittc = distance / rel_vel;
      			}
      			ittc_values.push_back(ittc);
		}
		angle += scan_msg->angle_increment;


	}
	//NOT SURE IF THIS IS THE BEST WAY TO FIND THIS
	double min_ittc = *std::min_element(ittc_values.begin(), ittc_values.end());
    	RCLCPP_INFO(this->get_logger(), "Minimum ITTC: %f seconds", min_ittc);
	/// TODO: publish drive/brake message
	
	if (min_ittc < ittc_threshold){
      		RCLCPP_WARN(this->get_logger(), "Low ITTC detected! Stopping the vehicle.");

      		ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      		drive_msg.drive.speed = 0.0;
      		ackermann_pub_->publish(drive_msg);
	}

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

