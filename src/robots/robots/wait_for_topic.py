#!/usr/bin/env python3
"""
One-shot node: subscribe to a topic and exit 0 when the first message is received.
Used by moon_bringup to start relay/localisation only after odom/scan data is published.
"""

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WaitForTopicNode(Node):
    def __init__(self):
        super().__init__("wait_for_topic")
        self.declare_parameter("topic", "/Moon/odom")
        self.declare_parameter("message_type", "Odometry")
        self.declare_parameter("timeout", 30.0)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        message_type = self.get_parameter("message_type").get_parameter_value().string_value
        t = self.get_parameter("timeout").get_parameter_value()
        try:
            timeout_sec = t.double_value if t.type == 3 else float(t.string_value or "30.0")
        except (TypeError, ValueError):
            timeout_sec = 30.0

        self.topic = topic
        self.timeout_sec = timeout_sec
        self.received = False

        if message_type in ("odometry", "Odometry", "nav_msgs/msg/Odometry"):
            self.sub = self.create_subscription(Odometry, topic, self._cb, 10)
        elif message_type in ("laser_scan", "LaserScan", "sensor_msgs/msg/LaserScan"):
            self.sub = self.create_subscription(LaserScan, topic, self._cb, 10)
        else:
            self.get_logger().error(f"Unsupported message_type: {message_type}")
            raise SystemExit(2)

        self.get_logger().info(
            f"Waiting for first message on '{topic}' (timeout {timeout_sec}s)..."
        )
        self.timer = self.create_timer(timeout_sec, self._timeout_cb)

    def _cb(self, msg):
        if self.received:
            return
        self.received = True
        self.get_logger().info(f"Received data on '{self.topic}' — starting next stage.")
        raise SystemExit(0)

    def _timeout_cb(self):
        if self.received:
            return
        self.get_logger().error(
            f"Timeout: no message on '{self.topic}' after {self.timeout_sec}s."
        )
        raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForTopicNode()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        rclpy.shutdown()
        sys.exit(e.code if e.code is not None else 0)
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
