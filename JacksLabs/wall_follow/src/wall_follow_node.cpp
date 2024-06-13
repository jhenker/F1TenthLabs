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
	 drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,10);
	 range_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic,10, std::bind(&WallFollow::scan_callback,this, std::placeholders::_1));
	previous_time_ = this->now().seconds();
        // TODO: create ROS subscribers and publishers
    }

private:
    // PID CONTROL PARAMS
    double kp = 9; 
    double kd = 0.05;
    double ki = 0;
    double servo_offset = 0.0;
    double prev_error_ = 0.0;
    //double error = 0.0
    double integral = 0.0;
    double previous_time_;
    double ldistance;
    rclcpp::Node::SharedPtr nh;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr range_subscription_;

    


    double get_range(const float* range_data, float angle, float angle_min, float angle_inc,double size)
	    
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
	int index = static_cast<int>((angle - angle_min) / angle_inc);
	 

        if(std::isinf(range_data[index]) || std::isnan(range_data[index])|| range_data[index]<0){
		    return 4;
	}
        // TODO: implement
	if (index >=0 && index < size){
		return range_data[index];
	}
	else{
		return -1;
	}
    }

    double get_error(double* range_arr, double desire_dist, double theta_rad, double * alpha)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        *alpha = std::atan((range_arr[0]*std::cos(theta_rad) - range_arr[1])/range_arr[0] * std::sin(theta_rad));

        double distance = range_arr[1]*std::cos(*alpha);
        double lookahead_distance = 0.5;

        ldistance = distance +  lookahead_distance*std::sin(*alpha);

	return desire_dist - ldistance;

       
        
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
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();


	double current_time = this->now().seconds();
	double dt = current_time - previous_time_;

	if (dt<= 0.0){
		dt = 1e-3;
	}
	integral += error *dt;
	double derivative = (error - prev_error_) / dt;

	angle = kp * error + ki * integral + kd * derivative;

	prev_error_ = error;
	previous_time_ = current_time;


        // TODO: fill in drive message and publish
	//
	//
	drive_msg.drive.steering_angle = -angle;
	drive_msg.drive.speed = velocity;
	drive_publisher_->publish(drive_msg);
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
        double error = 0.0; // TODO: replace with error calculated by get_error()
        //double velocity = 0.0; // TODO: calculate desired car velocity based on error
	double angle_min = scan_msg->angle_min;
	double angle_inc = scan_msg->angle_increment;
	double angle1 = 90.0;
	double angle2 = 60.0;
	double target_angle1_rad = angle1*M_PI/180.0;
	double target_angle2_rad = angle2*M_PI/180.0;


	double theta = angle1 - angle2;

	double theta_rad = theta * M_PI / 180.0;
	
	double size = scan_msg->ranges.size();

	double rangea = get_range(scan_msg->ranges.data(), target_angle1_rad, angle_min, angle_inc,size);
	double rangeb = get_range(scan_msg->ranges.data(), target_angle2_rad, angle_min, angle_inc,size);
	
	double ranges[] = {rangea,rangeb};

	double desired_distance = 0.8;
	double alpha = 0.0;
	error = get_error(ranges, desired_distance,theta_rad,&alpha);
	//use lookahead distance or current distance?
	alpha = ((alpha*180.0)/M_PI);
	RCLCPP_INFO(this->get_logger(),"What is alpha? alpha is: '%f'",alpha);

        // TODO: actuate the car with PID
	if (alpha<14 && alpha >-14){
	pid_control(error,1.5);
	}
	else if ((alpha>14 && alpha <20) || (alpha <-14 && alpha > -20)){
		pid_control(error,1.0);
	}
	else{
		pid_control(error,0.5);
	}
    }

};
int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
