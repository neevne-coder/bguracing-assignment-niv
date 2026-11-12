import os

import pandas as pd
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

CSV_PATH = os.environ.get("CSV_PATH", "/data/BrandsHatchLayout.csv")


def make_path_msg(points, frame_id):
    """Convert an Nx2 array of (x, y) points into a nav_msgs/Path."""
    msg = Path()
    msg.header.frame_id = frame_id
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        msg.poses.append(pose)
    return msg


class TrackPublisher(Node):
    def __init__(self):
        super().__init__("track_publisher")

        # transient_local: late subscribers still receive the last message
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.left_pub = self.create_publisher(Path, "track/left", qos)
        self.right_pub = self.create_publisher(Path, "track/right", qos)

        df = pd.read_csv(CSV_PATH).dropna()
        left = df[df["side"] == "left"][["x", "y"]].to_numpy()
        right = df[df["side"] == "right"][["x", "y"]].to_numpy()

        self.left_pub.publish(make_path_msg(left, "map"))
        self.right_pub.publish(make_path_msg(right, "map"))
        self.get_logger().info(
            f"Published {len(left)} left and {len(right)} right boundary points"
        )


def main():
    rclpy.init()
    node = TrackPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()