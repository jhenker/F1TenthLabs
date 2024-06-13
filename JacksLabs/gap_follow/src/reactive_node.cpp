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
    double safety_bubble_size = 0.75;

    void safety_bubble(std::vector<float>& ranges, double angle_increment){
	float min_dist = 0.0;
	int min_index = 0;


	for (size_t i = 0; i<ranges.size(); ++i){
		if (ranges[i]<ranges[min_index]){
			min_index = i;
		}
	}

	min_dist = ranges[min_index];

	float one_index_increment = min_dist * sin(angle_increment);
	float cur_distance = 0.0;
	int cur_index =0;

	while ((cur_distance < (safety_bubble_size/2.0))&& (size_t(min_index+cur_index) < ranges.size()) && min_index - cur_index  >= 0){
		ranges[min_index + cur_index] = 0.0;
		ranges[min_index - cur_index] = 0.0;
		cur_index++;
		cur_distance += one_index_increment;
	}
	return;


    }
    void preprocess_lidar(std::vector<float>& ranges,size_t window_size,double angle, float angle_min, float angle_inc)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
	//
	//
	
	 size_t angle_above_below = (angle * M_PI)/180;
	
	 int indexOne = (angle_above_below - angle_min)/angle_inc;
	 int indexTwo = (-angle_above_below - angle_min)/angle_inc;
	 

	 for (size_t k = static_cast<size_t>(indexOne); k<ranges.size(); ++k){
		 ranges[k] = ranges[k];
	 }
	 for (int z = indexTwo; z>0; --z){
		 ranges[z] = ranges[z];
	 }
		

	    size_t half_window = window_size / 2;
	    int count = 0;
	    for (size_t i =0; i<ranges.size(); ++i){

		    if (ranges[i] >= 0.0 && ranges[i] <= 4){

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
	int current_start=0;
	int max_start=0;
	int current_length = 0;
	int max_length=0;
	int max_end=0;
	for (size_t i = 0; i<ranges.size(); i++){
		if (ranges[i] > 0.01){
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

	return;
    }
    
    int find_best_point(std::vector<float> ranges,int indice[2])
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
	int start_i = indice[0];
	int end_i = indice[1];
	//int index = 0;
	double long_range = 0.0;
	for (int i = start_i; i<=end_i; ++i){
		if (ranges[i] > long_range){
			long_range = ranges[i];
		//	index = i;
		}
	}
	
	//RCLCPP_INFO(this->get_logger(,"longest range = '%f'",long_range);

	float range_thresh = long_range - 0.5;

	int reduced_range_start = -1;
	int reduced_range_end = -1;
	int max_length = 0;
	int current_length = 0;
	int current_start_index = -1;

	for (int i = start_i; i<end_i; i++){
	   if (ranges[i] > range_thresh){
		if (current_length==0){
		 current_start_index = i;
		}
		current_length++;
		if (current_length >= max_length){
			max_length = current_length;
	 		reduced_range_start = current_start_index;
			reduced_range_end = i;
		}

	   }else{
		   current_length = 0;
	   }



	}
	RCLCPP_INFO(this->get_logger(),"get stuff '%f'",reduced_range_start);
/*
	int end_index;
	ranges[0] = 0.0;
	end_index = (end_i - start_i) / 2;
	for (int i = 0; i<end_index; ++i){
		start_i++;
	}
*///^^look for midpoint
	int y = reduced_range_end - (max_length/2.0);
	return y;

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
	    preprocess_lidar(ranges,10,90,scan_msg->angle_min,scan_msg->angle_increment);

	    safety_bubble(ranges,scan_msg->angle_increment);
/*
	    size_t minIndex = 0;
	    for (size_t i = 0; i<ranges.size(); ++i){
		    if (ranges[i]<ranges[minIndex]){
			    minIndex = i;
		    }
	    }
	    if (ranges[minIndex] <4){
	    size_t rangeSize = 100;
	    //eliminate all points in the bubble (set them to zero)
//	    double minIndexAngle = scan_msg->angle_min + (minIndex * scan_msg->angle_increment);
//	    double minIndexAnglePlus = (30 * M_PI)/180;
//	    double the_angle;
//	    if (minIndexAngle < 0){
//		    the_angle = minIndexAngle - minIndexAnglePlus;
//	    }
//	    else{
//		    the_angle = minIndexAngle + minIndexAnglePlus;
//	    }
//	    double the_hypotenuse = ranges[the_angle];
//	    double the_distance = the_hypotenuse * std::sin(minIndexAnglePlus);
//
//
//
*///	    if (the_hypotenuse > 5)
/*		
	    if (ranges[minIndex] < 1.5 && ranges[minIndex]	    
	    size_t startIndex = (minIndex >= rangeSize) ? minIndex - rangeSize : 0;
	    size_t endIndex = (minIndex + rangeSize < ranges.size()) ? minIndex + rangeSize : ranges.size() - 1;
	    for (size_t j = startIndex; j<endIndex; ++j){
		    ranges[j]=0.0;
	    }

	   
	    
	    }
*/   /// ^^^MY terrible attempts at figuring out the safety bubble problem
						                            
        // Find max length gap 
	    int indice[2] = {0,0};
	    find_max_gap(ranges,indice);\
        // Find the best point in the gap 
	    int best_point = find_best_point(ranges,indice);
	    RCLCPP_INFO(this->get_logger(),"best point: '%d'",best_point);
        // Publish Drive message
	    double angle_increment = scan_msg->angle_increment;
	    double angle_to_target = (best_point * angle_increment) +scan_msg->angle_min;
	   
	    steerToTarget(angle_to_target,0.75);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
