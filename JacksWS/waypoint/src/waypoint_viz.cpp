#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fstream>
#include <sstream>
#include <string>

class WaypointViz : public rclcpp::Node
{
public:
	WaypointViz() : Node ("waypoint_viz")
	{
		publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoints",10);

		publishWaypointsFromCSV("src/waypoint/logs/wp-2024-06-14-19-26-16.csv");
		//PUT SOMETHING HERE ^^^
	}

private:

	void publishWaypointsFromCSV(const std::string& filePath)
	{
		std::vector<double> vector;
		int id = 0;

		std::ifstream file(filePath);
		if (!file.is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file");
			return;
		}
		std::string line;
		while (std::getline(file, line))
		{
			std::istringstream iss(line);
			std::string token;
			std::vector<double> values;

			while(std::getline(iss,token,','))
			{
				try
				{
values.push_back(std::stod(token));
				}
				catch (const  std::invalid_argument &e)
				{
					RCLCPP_ERROR(this->get_logger(), "Invalid number in CSV line: %s",line.c_str());
					continue;
				}
			}
			if (values.size() != 4){
				RCLCPP_ERROR(this->get_logger(), "Error parsing CSV line: %s", line.c_str());
				continue;
			}
		
		double x = values[0];
		double y = values[1];
using namespace std;


		auto marker = std::make_unique<visualization_msgs::msg::Marker>();
		marker->header.frame_id = "map";
		marker->header.stamp = this->now();
		marker->ns = "waypoints";
		marker->id = id++;
		marker->type = visualization_msgs::msg::Marker::SPHERE;
		marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.position.x = x;
		marker->pose.position.y = y;
		marker->scale.x = 0.2;
		marker->scale.y = 0.2;
		marker->scale.z = 0.4;
		marker->color.a = 1.0;
		marker->color.r = 0.0;
		marker->color.g = 1.0;
		marker->color.b = 0.0;

		publisher_->publish(std::move(marker));
		RCLCPP_INFO(this->get_logger(), "Published waypoint: %f, %f",x,y);
		}
	}
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;

};
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<WaypointViz>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
