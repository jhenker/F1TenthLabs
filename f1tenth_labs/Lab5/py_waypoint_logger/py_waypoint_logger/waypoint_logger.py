import rclpy
from rclpy.node import Node
import numpy as np
import atexit
import tf_transformations
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from nav_msgs.msg import Odometry
import os

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('py_waypoint_logger')
        self.subscriber = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            10
        )

        home = expanduser('~')
        log_dir = os.path.join(home, 'sim_ws', 'way_point_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.filepath = strftime(os.path.join(log_dir, 'wp-%Y-%m-%d-%H-%M-%S.csv'), gmtime())
        self.file = open(self.filepath, 'w')

        atexit.register(self.shutdown)
        self.get_logger().info('Saving waypoints to: ' + self.filepath)

        # Create a timer to call the logging function every 2 seconds
        self.timer = self.create_timer(1.5, self.log_waypoint)

        # Variable to store the latest data
        self.latest_data = None

    def save_waypoint(self, data):
        self.latest_data = data


    def log_waypoint(self):
        if self.latest_data is not None:
            quaternion = np.array([
                self.latest_data.pose.pose.orientation.x, 
                self.latest_data.pose.pose.orientation.y, 
                self.latest_data.pose.pose.orientation.z, 
                self.latest_data.pose.pose.orientation.w
            ])

            euler = tf_transformations.euler_from_quaternion(quaternion)
            speed = LA.norm(np.array([
                self.latest_data.twist.twist.linear.x, 
                self.latest_data.twist.twist.linear.y, 
                self.latest_data.twist.twist.linear.z
            ]), 2)

            if self.latest_data.twist.twist.linear.x > 0.:
                self.get_logger().info(f'Linear x: {self.latest_data.twist.twist.linear.x}')

            self.file.write('%f, %f, %f, %f\n' % (
                self.latest_data.pose.pose.position.x,
                self.latest_data.pose.pose.position.y,
                euler[2],
                speed
            ))

    def shutdown(self):
        self.file.close()
        self.get_logger().info('File closed, goodbye!')

def main(args=None):
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    rclpy.spin(waypoint_logger)
    waypoint_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

