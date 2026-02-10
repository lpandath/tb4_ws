#!/usr/bin/env python3
"""
Odometry Relay Node for TurtleBot4 with namespaces.

This node relays odometry from a source topic (typically /odom without namespace)
to a target topic within the robot's namespace (e.g., /Moon/odom).

It handles QoS conversion: subscribes with BEST_EFFORT (matching TurtleBot4 driver)
and publishes with RELIABLE (for navigation stack compatibility).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__("odom_relay")

        # Declare parameters
        self.declare_parameter("source_topic", "/odom")
        self.declare_parameter(
            "target_topic", "odom"
        )  # relative -> becomes /Moon/odom in namespace

        source_topic = self.get_parameter("source_topic").value
        target_topic = self.get_parameter("target_topic").value

        # Get namespace for logging
        namespace = self.get_namespace().strip("/")

        # QoS that matches TurtleBot4's odometry publisher (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Subscribe to source odometry (typically non-namespaced /odom)
        self.subscription = self.create_subscription(
            Odometry, source_topic, self.odom_callback, sensor_qos
        )

        # Publish to target topic (relative, becomes /Moon/odom in namespace)
        # Use RELIABLE for navigation stack
        self.publisher = self.create_publisher(Odometry, target_topic, 10)

        self.message_count = 0
        self.get_logger().info(f"Odom relay started for namespace: {namespace}")
        self.get_logger().info(f"Subscribing to: {source_topic}")
        self.get_logger().info(
            f"Publishing to: {target_topic} (-> /{namespace}/{target_topic})"
        )

    def odom_callback(self, msg: Odometry):
        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info("First odometry message received and relayed!")
        elif self.message_count % 100 == 0:
            self.get_logger().debug(f"Relayed {self.message_count} odometry messages")

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
