#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import threading
import datetime
import os

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Buffer for the latest position
        self._latest_position = None
        self._lock = threading.Lock()

        # Create log file in project root with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Get the project root (4 directories above this script)
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
        self.log_path = os.path.join(project_root, f"drone_position_{timestamp}.log")
        self.get_logger().info(f"Logging drone position to {self.log_path}")

        # QoS profile for best effort
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )

        # Timer to write position every 2 seconds
        self.create_timer(2.0, self.write_position_to_file)

    def position_callback(self, msg):
        # Store the latest position thread-safely
        with self._lock:
            self._latest_position = msg

    def write_position_to_file(self):
        # Write the latest position to the log file every 2 seconds
        with self._lock:
            msg = self._latest_position
        if msg is not None:
            pos = msg.pose.position
            log_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with open(self.log_path, "a") as f:
                f.write(f"{log_time},{pos.x},{pos.y},{pos.z}\n")

def main():
    rclpy.init()
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()