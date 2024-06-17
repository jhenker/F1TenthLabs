import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv
import os

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.publisher = self.create_publisher(Marker, 'waypoints_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        self.filepath = self.get_filepath()
        self.waypoints = self.load_waypoints()

    def get_filepath(self):
        home = os.path.expanduser('~')
        log_dir = os.path.join(home, 'sim_ws', 'way_point_logs')
        csv_files = [f for f in os.listdir(log_dir) if f.endswith('.csv')]
        if not csv_files:
            self.get_logger().error('No CSV file found in log directory.')
            return None
        latest_file = max(csv_files, key=lambda f: os.path.getctime(os.path.join(log_dir, f)))
        return os.path.join(log_dir, latest_file)

    def load_waypoints(self):
        waypoints = []
        if self.filepath:
            with open(self.filepath, 'r') as file:
                reader = csv.reader(file)
                for row in reader:
                    x, y, theta, speed = map(float, row)
                    waypoints.append((x, y, theta, speed))
        return waypoints

    def publish_waypoints(self):
        if not self.waypoints:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for waypoint in self.waypoints:
            p = Point()
            p.x = waypoint[0]
            p.y = waypoint[1]
            p.z = 0.0
            marker.points.append(p)

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

