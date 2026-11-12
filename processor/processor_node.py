import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path

from track_analysis import analyze

OUT_DIR = os.environ.get("OUT_DIR", "/output")


def path_to_array(msg):
    """Convert a nav_msgs/Path into an Nx2 numpy array."""
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])


class TrackProcessor(Node):
    def __init__(self):
        super().__init__("track_processor")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.left = None
        self.right = None
        self.done = False
        self.create_subscription(Path, "track/left", self.on_left, qos)
        self.create_subscription(Path, "track/right", self.on_right, qos)
        self.get_logger().info("Waiting for track data...")

    def on_left(self, msg):
        self.left = path_to_array(msg)
        self.get_logger().info(f"Received left boundary: {len(self.left)} points")
        self.try_process()

    def on_right(self, msg):
        self.right = path_to_array(msg)
        self.get_logger().info(f"Received right boundary: {len(self.right)} points")
        self.try_process()

    def try_process(self):
        if self.left is None or self.right is None or self.done:
            return
        results = analyze(self.left, self.right, OUT_DIR)
        self.get_logger().info(f"Done. Results: {results}")
        self.done = True


def main():
    rclpy.init()
    node = TrackProcessor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()