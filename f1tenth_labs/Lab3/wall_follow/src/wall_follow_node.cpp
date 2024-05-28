#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
	laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
	 drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
	 prev_time = this->now().seconds();
    }

private:
    // PID CONTROL PARAMS
    // fine tune with the following values kp, kd, ki, a_ref_angle, L, ideal_distance
    double kp = 3;
    double kd = 0.01;
    double ki = 0.1; //if things get weird swap this to 0
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double prev_time = 0.0;

    //ASUMED NUMBERS
    double infinity_replacement = 4.0;
    double b_ref_angle = (90.0 * M_PI) / 180.0;
    double a_ref_angle = (45.0 * M_PI) / 180.0;
    double L = 1; //the amount the car looks ahead to predict the movement
    double ideal_distance = 1; // the cars ideal distance from the wall in meters
    

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    double get_range(const float* range_data, double angle, double angle_min, double angle_max, double angle_increment)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement
	if(angle > angle_max || angle < angle_min){
		RCLCPP_WARN(this->get_logger(), "Angle not in accaptable rance");
		return 0.0;
	}

	int index = static_cast<int>((angle - angle_min)/angle_increment);
	double distance = range_data[index];
	//RCLCPP_INFO(this->get_logger(), "Min angle: '%f'", angle_min);
	if(std::isnan(distance) || std::isinf(distance) || distance <= 0.0){
		distance = infinity_replacement;
		RCLCPP_INFO(this->get_logger(), "Using infinity replacement");
	}

        return distance;
    }

    double get_error(double* range_data, double dist, double theta, double *alpha_ptr)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
	double a = range_data[0];
	double b = range_data[1];

	double alpha = atan( ((a * cos(theta)) - b) / (a*sin(theta)) );
	*alpha_ptr = alpha;

	double D_t = b * cos(alpha);
	double D_t1 = D_t + (L * sin(alpha));
	double error = dist - D_t1;
        return error;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

	double current_time = this->now().seconds();
        double dt = current_time - prev_time;

	if (dt <= 0.0) {
            dt = 1e-3;  // Avoid division by zero
        }

	integral += error * dt;
        double derivative = (error - prev_error) / dt;

        double angle = kp * error + ki * integral + kd * derivative;

	prev_error = error;
        prev_time = current_time;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
	drive_msg.drive.steering_angle = -angle;
	drive_msg.drive.speed = velocity;
	drive_pub_->publish(drive_msg);

        // TODO: fill in drive message and publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
	double theta = (b_ref_angle - a_ref_angle);

	double dist_a = get_range(scan_msg->ranges.data(), a_ref_angle, scan_msg->angle_min, scan_msg->angle_max, scan_msg->angle_increment);
	double dist_b = get_range(scan_msg->ranges.data(), b_ref_angle, scan_msg->angle_min, scan_msg->angle_max, scan_msg->angle_increment);
       
	double dist_array[2] = {dist_a, dist_b};
	
	double alpha = 0.0;

	double error = get_error(dist_array, ideal_distance, theta, &alpha); // TODO: replace with error calculated by get_error()
        double velocity = 1.1; // TODO: calculate desired car velocity based on error
	alpha = (alpha * 180.0) / M_PI;
	
	if(alpha <= 10 && alpha >= -10){
		velocity = 1.5;
		//RCLCPP_INFO(this->get_logger(), "I AM SPEEEEEEEEEEEEEEEED");
	}else if(alpha <= 20 && alpha >= -20){
		velocity = 1.0;
		//RCLCPP_INFO(this->get_logger(), "Monerate Momentum");
	}else{
		velocity = 0.5;
		//RCLCPP_INFO(this->get_logger(), "super slow");
	}
        // TODO: actuate the car with PID
	pid_control(error,velocity);
	
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
