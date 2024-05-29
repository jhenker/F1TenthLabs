#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <functional>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
	//
	laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic,10,std::bind(&ReactiveFollowGap::lidar_callback, this,std::placeholders::_1));
	drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

    void preprocess_lidar(std::vector<float>& ranges,size_t window_size)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
	    size_t half_window = window_size / 2;
	    int count = 0;
	    for (size_t i =0; i<ranges.size(); ++i){

		    if (ranges[i] >= 0.0 && ranges[i] <= 7){

			    size_t start_index = std::max(static_cast<int>(i) - static_cast<int>(half_window), 0 );
			    size_t end_index = std::min(i + half_window, ranges.size() -1);
			    double sum = 0.0;
			    count = 0;
			    for (size_t j = start_index; j <=end_index; ++j){
				    sum+=ranges[j];
				    count++;
			    }
			    if(count>0){
				    float mean = sum/count;
				    ranges[i] = mean;
			    
			    }
		    }
			    else{
				    ranges[i] = 4;
			    }


      
   		
	    }
	    return;
    }

    void find_max_gap(std::vector<float> ranges, int indice[2])
    {   
        // Return the start index & end index of the max gap in free_space_ranges
	int current_start;
	int max_start;
	int current_length;
	int max_length;
	int max_end;
	for (size_t i = 0; i<ranges.size(); i++){
		if (ranges[i] > 0){
			if (current_length == 0){
		
				current_start = i;
			}
			current_length++;
			if (current_length > max_length){
				max_length = current_length;
				max_start = current_start;
				max_end = i;
			}	
		}
		else{
			current_length=0;
			
		}
	indice[0] = max_start;
	indice[1] = max_end;	

    }
	RCLCPP_INFO(this->get_logger(),"what is the index: '%d' other one? : '%d'",indice[0],indice[1]);
	return;
    }

    int find_best_point(int indice[2])
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        //COULD ALSO CHOOS MIDPOINT???
	int start_i = indice[0];
	int end_i = indice[1];
	
	RCLCPP_INFO(this->get_logger(),"start index = '%d', end index = '%d'",start_i,end_i);
	//double long_range = 0.0;
	//for (int i = start_i; i<=end_i; ++i){
	//	if (ranges[i] > long_range){
	//		long_range = ranges[i];
	//		index = i;
	//	}
	//}//
	int end_index;
	end_index = (end_i - start_i) / 2;
	for (int i = 0; i<end_index; ++i){
		start_i++;
	}

			return start_i;
    }
    void steerToTarget(double angle_to_target, double speed){
	    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
	    drive_msg.drive.speed = speed;
	    drive_msg.drive.steering_angle = angle_to_target;


	    drive_publisher_->publish(drive_msg);
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
	
	    std::vector<float> ranges = scan_msg->ranges;
	    preprocess_lidar(ranges,6);
	    size_t minIndex = 0;
	    for (size_t i = 0; i<ranges.size(); ++i){
		    if (ranges[i]<ranges[minIndex]){
			    minIndex = i;
		    }
	    }
	    size_t rangeSize = 30;    
	    size_t startIndex = (minIndex >= rangeSize) ? minIndex - rangeSize : 0;
	    size_t endIndex = (minIndex + rangeSize < ranges.size()) ? minIndex + rangeSize : ranges.size() - 1;
	    for (size_t j = startIndex; j<endIndex; ++j){
		    ranges[j]=0.0;
	    }
        // Eliminate all points inside 'bubble' (set them to zero) 
	    



	    RCLCPP_INFO(this->get_logger(),"deleted the bubble");
        // Find max length gap 
	    int indice[2] = {0,0};
	    find_max_gap(ranges,indice);
	    RCLCPP_INFO(this->get_logger(),"this '%d'",indice[0]);
        // Find the best point in the gap 
	    int best_point = find_best_point(indice);
	    RCLCPP_INFO(this->get_logger(),"best point: '%d'",best_point);
        // Publish Drive message
	    double angle_increment = scan_msg->angle_increment;
	    double angle_to_target = (best_point * angle_increment) +scan_msg->angle_min;
	    RCLCPP_INFO(this->get_logger(),"angle target : '%f'",angle_to_target);
	    steerToTarget(angle_to_target,0.6);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
