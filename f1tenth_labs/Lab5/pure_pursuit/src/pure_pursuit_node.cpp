#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>

using namespace std;

class PurePursuit : public rclcpp::Node
{
private:
    vector<vector<double>> waypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    double lookahead_distance_ = 4.0;
    double max_steering_angle_ = 0.5;
    double max_speed_ = 1.5;
    string file_name = "wall_follow_1.csv";

    void load_waypoints(const string &filepath)
    {
	RCLCPP_INFO(this->get_logger(), "About to Loading File");
        ifstream file(filepath);
        string line;
        while (getline(file, line))
        {
	    RCLCPP_INFO(this->get_logger(), "Loading File");
            stringstream ss(line);
            string token;
            vector<double> waypoint;
            while (getline(ss, token, ','))
            {
                waypoint.push_back(stod(token));
            }
            waypoints.push_back(waypoint);
        }
        file.close();
    }

    void visualize_target_waypoint(double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        //marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub_->publish(marker);
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
    // void pose_callback(const rclcpp::Subscription<const geometry_msgs::msg::PoseStamped>::SharedPtr pose_msg)
    // void pose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped_<std::allocator<void> >> pose_msg)
    {
        if (waypoints.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No waypoints loaded.");
            return;
        }

        double robot_x = pose_msg->pose.pose.position.x;
        double robot_y = pose_msg->pose.pose.position.y;

        double min_distance = numeric_limits<double>::max();
        int closest_waypoint_index = -1;
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            double distance = sqrt(pow(waypoints[i][0] - robot_x, 2) + pow(waypoints[i][1] - robot_y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_waypoint_index = i;
            }
        }

        int lookahead_index = closest_waypoint_index;
        for (size_t i = closest_waypoint_index; i < waypoints.size(); ++i)
        {
            double distance = sqrt(pow(waypoints[i][0] - robot_x, 2) + pow(waypoints[i][1] - robot_y, 2));
            if (distance > lookahead_distance_)
            {
                lookahead_index = i;
                break;
            }
        }

        if (lookahead_index == -1)
        {
            RCLCPP_WARN(this->get_logger(), "No valid lookahead waypoint found.");
            return;
        }

        double lookahead_x = waypoints[lookahead_index][0];
        double lookahead_y = waypoints[lookahead_index][1];

        double dx = lookahead_x - robot_x;
        double dy = lookahead_y - robot_y;
        double heading = pose_msg->pose.pose.orientation.z; //MIGHT NEED TO NOT BE QUATERNIAN
        double transformed_x = cos(heading) * dx + sin(heading) * dy;
        double transformed_y = -sin(heading) * dx + cos(heading) * dy;

        double curvature = 2 * transformed_y / pow(lookahead_distance_, 2);
        double steering_angle = atan(curvature);

        if (steering_angle > max_steering_angle_)
        {
            steering_angle = max_steering_angle_;
        }
        else if (steering_angle < -max_steering_angle_)
        {
            steering_angle = -max_steering_angle_;
        }

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = max_speed_;
        drive_pub_->publish(drive_msg);

        visualize_target_waypoint(transformed_x, transformed_y);
    }

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        this->declare_parameter("lookahead_distance", 2.0);
        this->declare_parameter("max_steering_angle", 0.4189); // ~24 degrees
        this->declare_parameter("max_speed", 2.0);
        this->declare_parameter("waypoints_file", std::getenv("HOME") + std::string("/sim_ws/way_point_logs/") + std::string(file_name));

        this->get_parameter("lookahead_distance", lookahead_distance_);
        this->get_parameter("max_steering_angle", max_steering_angle_);
        this->get_parameter("max_speed", max_speed_);

        std::string waypoints_file;
        this->get_parameter("waypoints_file", waypoints_file);
        load_waypoints(waypoints_file);

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/target_marker", 10);
    }

    ~PurePursuit() {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
