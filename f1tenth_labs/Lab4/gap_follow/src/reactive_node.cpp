#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <memory>
#include <algorithm>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
	laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
         drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    
    double safety_bubble_size = 0.5;
    float max_range_offset = 2;

    void preprocess_lidar(std::vector<float>& ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
	for (float& range : ranges) {
            if (range > 3.0) {
                range = 3.0;
            }
        }
	
	size_t back_edges = ranges.size() / 6;
	for(size_t i = 0; i < ranges.size(); i++){
		if(i < back_edges || i > (ranges.size() - back_edges)){
			//ranges[i] = 0.0;
		}
	}
	ranges[0];
        return;
    }

    void update_safety_bubble(std::vector<float>& ranges, double angle_increment){
	//find closest distance and index
	float min_dist = 0.0;
	int min_index = 0;
	for(size_t i = 0; i < ranges.size(); i++){
		if(ranges[i] < ranges[min_index]){
			min_index = i;
			//RCLCPP_INFO(this->get_logger(),"Updating Min Index to %d", min_index);
		}
	
	}
	min_dist = ranges[min_index];
	
	//zero out the array within the distance 
	float one_index_increment = min_dist * tan(angle_increment);
	    //update from sin to tan. Tan gets the distance along the 
	float cur_distance = 0.0;
	int cur_index = 0;
	while((cur_distance < (safety_bubble_size / 2.0)) && (size_t(min_index+cur_index) < ranges.size()) && (min_index-cur_index >= 0)){
		//RCLCPP_INFO(this->get_logger(),"While %f is less than %f, Cur_INDEX IS %d",cur_distance, safety_bubble_size / 2.0, cur_index);
		ranges[min_index + cur_index] = 0.0;
		ranges[min_index - cur_index] = 0.0;
		//RCLCPP_INFO(this->get_logger(),"Ranges that set to zero are %f and %f", ranges[min_index + cur_index], ranges[min_index-cur_index]);
		cur_index++;
		cur_distance += one_index_increment;
	}
	//loop through half the size of safety bubble zeroing the array

	return;
    }

    void find_max_gap(std::vector<float> ranges, int (&indice)[3])
    {   
        // Return the start index & end index of the max gap in free_space_ranges
	int start_cur_gap = -1;
	int start_max_gap = -1;
	int len_max_gap = 0;
	int len_cur_gap = 0;
	int max_gap_end = -1;
	for(size_t i = 0; i < ranges.size(); i++){
		//RCLCPP_INFO(this->get_logger(),"range of i:%d '%f'",i ,ranges[i]);
		if(ranges[i] > 0){
			if(len_cur_gap == 0){
				start_max_gap = i;
			}

			len_cur_gap++;
			if(len_cur_gap > len_max_gap){
				len_max_gap = len_cur_gap;
				start_max_gap = start_cur_gap;
				max_gap_end = i;
			}
		}else{
			start_cur_gap = 0;
		}
	}

	indice[0] = start_max_gap;
	//if(len_max_gap == 0){
	//	indice[2] = ranges.size();
	//}else{
	indice[2] = max_gap_end;
	//}
        return;
    }

    void find_best_point(std::vector<float> ranges, int (&indice)[3])
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
	float furthest_point = 0.0;
	int furthest_point_index = 0;
	//RCLCPP_INFO(this->get_logger(), "Looping from %d to %d", indice[0], indice[2]);
	for(int i = indice[0]; i <= indice[2]; i++){
		//RCLCPP_INFO(this->get_logger(), "every point is %f", ranges[i]);
		if(ranges[i] > furthest_point){
			furthest_point = ranges[i];
			furthest_point_index = i;

		}
		//RCLCPP_INFO(this->get_logger(), "in the loop i = %d", i);
	}
	//RCLCPP_INFO(this->get_logger(), "Furthest point is %f at index %d", furthest_point, furthest_point_index);
	

	//loop through and set the range to the largest amount of points that are farther than updated_range_threshhold
	int reduced_range_start = -1;
	int reduced_range_end = -1;
	int max_length = 0;
    	int current_length = 0;
    	int current_start_index = -1;
	float range_thresh = furthest_point - max_range_offset;
	for(int i = indice[0]; i <= indice[2]; i++){
                //RCLCPP_INFO(this->get_logger(), "every point is %f", ranges[i]);
                if(ranges[i] > range_thresh){
        		if(current_length == 0){
				current_start_index = i;
			}
			current_length++;
			if(current_length > max_length){
				max_length = current_length;
				reduced_range_start = current_start_index;
				reduced_range_end = i;
			}

                }else{
			current_length = 0;
				
		}
                
        }

	//RCLCPP_INFO(this->get_logger(), "Furthest point range start %d end %d", reduced_range_start, reduced_range_end);

	indice[1] = reduced_range_end - (max_length / 2);

	//indice[1] = (indice[2] - indice[0]) / 2;

        return;
    }
	
    void publish_drive(int indice[3], float angle_increment, float min_angle){
	
	float angle = (indice[1] * angle_increment) + min_angle; //index * increment + min
	double velocity = 1;

	auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();                                                                                        
	drive_msg.drive.steering_angle = angle;                                                                                                              
	drive_msg.drive.speed = velocity;                                                                              
	drive_pub_->publish(drive_msg);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
	//int num_ranges = scan_msg->ranges.size();
	//RCLCPP_INFO(this->get_logger(), "About to Allocate");
	std::vector<float> ranges = scan_msg->ranges;
	//std::copy(scan_msg->ranges.begin(), scan_msg->ranges.end(), ranges.get());
	preprocess_lidar(ranges);
        /// TODO:
        // Find closest point to LiDAR
	//RCLCPP_INFO(this->get_logger(), "Working before any functions");	
        // Eliminate all points inside 'bubble' (set them to zero) 
	update_safety_bubble(ranges, scan_msg->angle_increment);
        // Find max length gap 
	int indices[3] = {0,0,0}; //first and last index are start and end of the window, middle index is the index of 
	//RCLCPP_INFO(this->get_logger(),"index 0:%d 1:%d 2:%d",indices[0], indices[1], indices[2]);
	find_max_gap(ranges, indices);
	// Find the best point in the gap 
	find_best_point(ranges, indices);
	//RCLCPP_INFO(this->get_logger(), "About to publish");
        // Publish Drive message
	//RCLCPP_INFO(this->get_logger(),"index 0:%d 1:%d 2:%d",indices[0], indices[1], indices[2]);
	publish_drive(indices, scan_msg->angle_increment, scan_msg->angle_min);
	//RCLCPP_INFO(this->get_logger(), "Published");
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
