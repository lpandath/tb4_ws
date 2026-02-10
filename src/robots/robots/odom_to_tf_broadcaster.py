#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFBroadcaster(Node):
    """Convert odometry messages to TF transforms and publish at fixed rate."""

    def __init__(self):
        super().__init__("odom_to_tf_broadcaster")

        # Get namespace from node namespace
        self.namespace = self.get_namespace().strip("/")

        # Message counter for diagnostics
        self.msg_count = 0

        # Store last known pose and its stamp for TF at correct time
        self.last_pose = None
        self.last_odom_stamp = None

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS profile to EXACTLY match TurtleBot4's odometry publisher
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # Subscribe to odometry with sensor-compatible QoS
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, odom_qos
        )

        # Publish odom->base_link TF at fixed 20 Hz rate (every 50ms)
        # This ensures continuous TF even when odom messages are sparse
        self.create_timer(0.05, self.publish_odom_tf)

        # Diagnostic timer - check if receiving odom
        self.create_timer(5.0, self.check_odom_status)

        # Resolve the full topic name for logging
        odom_topic = f"/{self.namespace}/odom" if self.namespace else "/odom"
        self.get_logger().info(
            f"Odometry to TF broadcaster started for namespace: {self.namespace}"
        )
        self.get_logger().info(
            f"Subscribing to: {odom_topic} with QoS: RELIABLE + TRANSIENT_LOCAL"
        )
        self.get_logger().info(
            f"Will publish TF: {self.namespace}/odom -> {self.namespace}/base_link at 20 Hz"
        )
        self.get_logger().info(
            f"NOTE: Static transforms (base_link->rplidar_link) are published by robot via /tf_static"
        )

    def check_odom_status(self):
        """Periodically check if we're receiving odom messages."""
        if self.msg_count == 0:
            self.get_logger().warn(
                f"WARNING: No odometry messages received yet! "
                f"Check that robot is publishing to /{self.namespace}/odom"
            )
        else:
            self.get_logger().info(f"Received {self.msg_count} odom messages so far")

    def odom_callback(self, msg: Odometry):
        """Store the latest odometry pose."""
        self.msg_count += 1
        if self.msg_count == 1:
            self.get_logger().info(
                "SUCCESS: First odometry message received! Publishing TF now."
            )
        elif self.msg_count % 500 == 0:
            self.get_logger().info(f"Odom OK: received {self.msg_count} messages")

        # Store the pose and stamp so TF is published at odom time (not "now")
        self.last_pose = msg.pose.pose
        self.last_odom_stamp = msg.header.stamp
        # Publish immediately at this stamp so TF buffer has transform at scan-relevant times
        self._send_odom_tf(self.last_pose, self.last_odom_stamp)

    def _send_odom_tf(self, pose, stamp):
        """Publish a single odom->base_link transform using CURRENT time."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # Use current time, not old stamp
        t.header.frame_id = f"{self.namespace}/odom" if self.namespace else "odom"
        t.child_frame_id = (
            f"{self.namespace}/base_link" if self.namespace else "base_link"
        )
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_odom_tf(self):
        """Publish odom->base_link TF at fixed rate using last known pose and stamp."""
        if self.last_pose is None or self.last_odom_stamp is None:
            return
        self._send_odom_tf(self.last_pose, self.last_odom_stamp)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()