import json
import os

import pandas as pd
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

TRACK_FILE = os.environ.get("TRACK_FILE", "/data/BrandsHatchLayout.csv")
CAR_FILE = os.environ.get("CAR_FILE", "/config/car.yaml")


def make_path_msg(points):
    """Turn a list of (x, y) points into a Path message."""
    msg = Path()
    msg.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        msg.poses.append(pose)
    return msg


class DataPublisher(Node):
    def __init__(self):
        super().__init__("data_publisher")
        # transient_local: a subscriber that starts later still receives the last message
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        left_pub = self.create_publisher(Path, "track/left", qos)
        right_pub = self.create_publisher(Path, "track/right", qos)
        car_pub = self.create_publisher(String, "car/config", qos)

        track = pd.read_csv(TRACK_FILE).dropna()
        left = track[track["side"] == "left"][["x", "y"]].to_numpy()
        right = track[track["side"] == "right"][["x", "y"]].to_numpy()
        with open(CAR_FILE) as f:
            config = yaml.safe_load(f)

        left_pub.publish(make_path_msg(left))
        right_pub.publish(make_path_msg(right))
        car_pub.publish(String(data=json.dumps(config)))
        self.get_logger().info(
            f"Published {len(left)} left points, {len(right)} right points and the car config")


def main():
    rclpy.init()
    node = DataPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()