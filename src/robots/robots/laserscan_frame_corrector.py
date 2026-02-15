#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class LaserScanFrameCorrector(Node):
    def __init__(self):
        super().__init__("laserscan_frame_corrector")

        # Declare parameters
        self.declare_parameter("namespace", "Moon")
        self.declare_parameter("input_topic", "scan")
        self.declare_parameter("output_topic", "scan_new")
        self.declare_parameter("input_frame", "rplidar_link")
        self.declare_parameter("output_frame", "rplidar_link")
        # Stamp scan with this node's clock ("now") so TF lookup matches odom->base_link (also "now").
        # Use when robot and laptop clocks differ (avoids circle artifact from timestamp mismatch).
        self.declare_parameter("use_receive_time", True)

        # Get parameters
        namespace = self.get_parameter("namespace").value
        self.input_frame = self.get_parameter("input_frame").value
        self.output_frame = f"{namespace}/{self.get_parameter('output_frame').value}"
        self.use_receive_time = self.get_parameter("use_receive_time").value

        # TurtleBot4 may use base_scan or laser_frame; accept all and normalize to our TF frame
        self._accepted_frames = {
            self.input_frame,
            self.output_frame,
            "base_scan",
            "laser_frame",
        }
        self._warned_frames = set()

        # Subscribe with BEST_EFFORT (matches robot's lidar publisher)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Publish with RELIABLE so AMCL and other nav2 nodes can receive scans
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            sub_qos,
        )
        self.publisher = self.create_publisher(
            LaserScan, "scan_new", pub_qos
        )

        self.get_logger().info(
            f"LaserScan frame corrector started for namespace: {namespace}"
        )
        self.get_logger().info(
            f"Converting frame: {self.input_frame} -> {self.output_frame}"
        )
        if self.use_receive_time:
            self.get_logger().info(
                "Scan stamp = receive time (laptop clock) so TF lookup matches odom TF."
            )

    def scan_callback(self, msg):
        # Set frame_id to our TF frame so costmap/RViz can transform correctly.
        incoming = msg.header.frame_id
        if incoming != self.output_frame:
            if incoming not in self._accepted_frames and incoming not in self._warned_frames:
                self._warned_frames.add(incoming)
                self.get_logger().warn(
                    f"Scan frame_id '{incoming}' not in TF tree; rewriting to {self.output_frame}"
                )
            msg.header.frame_id = self.output_frame

        # Use this node's clock so scan and odom->base_link TF share the same time base (fixes circle when robot/laptop clocks differ).
        if self.use_receive_time:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFrameCorrector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()