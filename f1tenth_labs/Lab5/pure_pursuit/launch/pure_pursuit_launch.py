from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',  # Replace with your pure pursuit package name
            executable='pure_pursuit_node',  # Replace with your pure pursuit executable name
            name='pure_pursuit_node',
            output='screen',
            #parameters=[
            #    {'lookahead_distance': 2.0},
            #    {'max_steering_angle': 0.4189},
            #    {'max_speed': 2.0},
            #    {'waypoints_file': '/home/user/sim_ws/way_point_logs/wall_follow_1.csv'}  # Adjust path as needed
            #]
        ),
        Node(
            package='py_waypoint_logger',  # Replace with your visualization package name
            executable='waypoint_visualizer',  # Replace with your visualization executable name
            name='visualization_node',
            output='screen'
        )
    ])

