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

    double lookahead_distance_ = 1.85;
    double max_steering_angle_ = 0.4;
    double max_speed_ = 8;
    string file_name = "gap_follow_1.csv";

    void load_waypoints(const string &filepath)
    {
        // RCLCPP_INFO(this->get_logger(), "About to Loading File");
        ifstream file(filepath);
        string line;
        while (getline(file, line))
        {
            // RCLCPP_INFO(this->get_logger(), "Loading File");
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
        // visualization_msgs::msg::Marker marker;
        auto marker = std::make_unique<visualization_msgs::msg::Marker>();
        marker->header.frame_id = "map";
        marker->header.stamp = this->now();
        marker->ns = "pure_pursuit";
        marker->id = 0;
        marker->type = visualization_msgs::msg::Marker::SPHERE;
        marker->action = visualization_msgs::msg::Marker::ADD;
        marker->pose.position.x = x;
        marker->pose.position.y = y;
        marker->pose.position.z = 0.0;
        marker->pose.orientation.x = 0.0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = 0.0;
        marker->pose.orientation.w = 1.0;
        marker->scale.x = 0.5;
        marker->scale.y = 0.5;
        marker->scale.z = 0.5;
        marker->color.a = 1.0;
        marker->color.r = 1.0;
        marker->color.g = 0.0;
        marker->color.b = 1.0;
        marker_pub_->publish(std::move(marker));
    }

    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
    {
        double robot_x = pose_msg->pose.pose.position.x;
        double robot_y = pose_msg->pose.pose.position.y;

        // Find the closest waypoint to the robot
        double min_distance = std::numeric_limits<double>::max();
        int closest_waypoint_index = -1;
        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            double distance = std::hypot(waypoints[i][0] - robot_x, waypoints[i][1] - robot_y);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_waypoint_index = i;
            }
        }

        // Ensure valid closest waypoint index
        if (closest_waypoint_index == -1)
        {
            RCLCPP_WARN(this->get_logger(), "No valid closest waypoint found.");
            return;
        }

        // Find the lookahead waypoint using interpolation
        int lookahead_index = closest_waypoint_index;
        double cumulative_distance = 0.0;
        double target_distance = lookahead_distance_;
        double segment_distance = 0.0;

        for (size_t i = closest_waypoint_index; i < waypoints.size() - 1; ++i)
        {
            segment_distance = std::hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1]);
            if (cumulative_distance + segment_distance > target_distance)
            {
                lookahead_index = i;
                break;
            }
            cumulative_distance += segment_distance;
        }

        if (segment_distance == 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Segment distance is zero, cannot interpolate.");
            return;
        }

        // Calculate interpolation ratio
        double remaining_distance = target_distance - cumulative_distance;
        double ratio = remaining_distance / segment_distance;

        double lookahead_x = waypoints[lookahead_index][0] + ratio * (waypoints[lookahead_index + 1][0] - waypoints[lookahead_index][0]);
        double lookahead_y = waypoints[lookahead_index][1] + ratio * (waypoints[lookahead_index + 1][1] - waypoints[lookahead_index][1]);

        // Transform the lookahead point to the robot's coordinate frame
        tf2::Quaternion q(
            pose_msg->pose.pose.orientation.x,
            pose_msg->pose.pose.orientation.y,
            pose_msg->pose.pose.orientation.z,
            pose_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double dx = lookahead_x - robot_x;
        double dy = lookahead_y - robot_y;
        double transformed_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
        double transformed_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

        // Calculate the curvature and the steering angle
        double curvature = 2 * transformed_y / std::pow(lookahead_distance_, 2);
        double steering_angle = std::atan(curvature);

        // Clamp the steering angle to the maximum limits
        if (steering_angle > max_steering_angle_)
        {
            steering_angle = max_steering_angle_;
        }
        else if (steering_angle < -max_steering_angle_)
        {
            steering_angle = -max_steering_angle_;
        }

        // Adjust speed based on steering angle
        double cur_speed = max_speed_;
        if (std::abs(steering_angle) >= 0.3)
        {
            cur_speed = 0.8;
        }
        else if (std::abs(steering_angle) >= 0.2)
        {
            cur_speed = 1.5;
        }

        // Publish the drive message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = cur_speed;
        drive_pub_->publish(drive_msg);

        visualize_target_waypoint(lookahead_x, lookahead_y);
    }

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        this->declare_parameter("lookahead_distance", lookahead_distance_);
        this->declare_parameter("max_steering_angle", lookahead_distance_); // ~24 degrees
        this->declare_parameter("max_speed", max_speed_);
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
