import json
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from std_msgs.msg import String

from racing_line import (TIRE_GRIP, make_corridor, make_line, speed_profile,
                         pedal_commands, optimize_line)
from outputs import save_outputs

OUT_DIR = os.environ.get("OUT_DIR", "/output")


def path_to_array(msg):
    """Turn a Path message into a numpy array with one (x, y) row per point."""
    return np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])


class RacingLineProcessor(Node):
    def __init__(self):
        super().__init__("racing_line_processor")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.left = None
        self.right = None
        self.config = None
        self.done = False
        self.create_subscription(Path, "track/left", self.on_left, qos)
        self.create_subscription(Path, "track/right", self.on_right, qos)
        self.create_subscription(String, "car/config", self.on_config, qos)
        self.get_logger().info("Waiting for track and car data...")

    def on_left(self, msg):
        self.left = path_to_array(msg)
        self.run_if_ready()

    def on_right(self, msg):
        self.right = path_to_array(msg)
        self.run_if_ready()

    def on_config(self, msg):
        self.config = json.loads(msg.data)
        self.run_if_ready()

    def run_if_ready(self):
        if self.done or self.left is None or self.right is None or self.config is None:
            return
        self.get_logger().info("All data received. Searching for the best racing line...")

        car = self.config["car"]
        margin = self.config["line"]["margin_m"]
        grip = TIRE_GRIP[self.config["tire"]["compound"]][self.config["track"]["surface"]]

        corridor = make_corridor(self.left, self.right, margin)        
        center_line = make_line(corridor, np.zeros(len(corridor["center"])))
        center_time = speed_profile(center_line, car, grip)[2]

        offsets = optimize_line(corridor, car, grip) 
        line = make_line(corridor, offsets)
        speed, distance, lap_time = speed_profile(line, car, grip)
        throttle, brake = pedal_commands(speed, distance, car)
        clearance = corridor["walls"].query(line)[0]  # distance of each line point to the nearest wall

        results = {
            "tire": self.config["tire"]["compound"],
            "surface": self.config["track"]["surface"],
            "grip": grip,
            "center_line_lap_time_s": round(center_time, 1),
            "racing_line_lap_time_s": round(lap_time, 1),
            "improvement_percent": round(100 * (center_time - lap_time) / center_time, 1),
            "line_length_m": round(float(distance.sum()), 1),
            "max_speed_kmh": round(float(speed.max() * 3.6), 1),
            "min_speed_kmh": round(float(speed.min() * 3.6), 1),
            "max_offset_m": round(float(np.abs(offsets).max()), 1),
            "min_distance_to_edge_m": round(float(clearance.min()), 2),
        }
        save_outputs(OUT_DIR, self.left, self.right, corridor, offsets, line, speed,
                     distance, throttle, brake, results)
        self.get_logger().info(f"Done. Results: {results}")
        self.done = True


def main():
    rclpy.init()
    node = RacingLineProcessor()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()