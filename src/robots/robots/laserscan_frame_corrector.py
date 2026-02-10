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

        # Get parameters
        namespace = self.get_parameter("namespace").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.input_frame = self.get_parameter("input_frame").value
        self.output_frame = f"{namespace}/{self.get_parameter('output_frame').value}"
        # TurtleBot4 may use base_scan or laser_frame; accept all and normalize to our TF frame
        self._accepted_frames = {
            self.input_frame,
            self.output_frame,
            "base_scan",
            "laser_frame",
        }
        self._warned_frames = set()

        # High-performance QoS for real-time sensor data
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # CORRECTED: Use relative topic names (no leading slash)
        # When node is in namespace 'Basin', 'scan' becomes 'Basin/scan'
        # When node is in namespace 'Moon', 'scan' becomes 'Moon/scan'

        self.subscription = self.create_subscription(
            LaserScan,
            "scan",  # CORRECTED: Relative topic - no slash
            self.scan_callback,
            qos_profile,
        )

        self.publisher = self.create_publisher(
            LaserScan, "scan_new", qos_profile  # CORRECTED: Relative topic - no slash
        )

        self.get_logger().info(
            f"LaserScan frame corrector started for namespace: {namespace}"
        )
        self.get_logger().info(f"Subscribing to: scan (will be {namespace}/scan)")
        self.get_logger().info(
            f"Publishing to: scan_new (will be {namespace}/scan_new)"
        )
        self.get_logger().info(
            f"Converting frame: {self.input_frame} -> {self.output_frame}"
        )

    def scan_callback(self, msg):
        # Always set frame_id to our TF frame so costmap/RViz can transform correctly.
        # Driver may send rplidar_link, base_scan, or laser_frame; our TF has Moon/rplidar_link.
        incoming = msg.header.frame_id
        if incoming != self.output_frame:
            if incoming not in self._accepted_frames and incoming not in self._warned_frames:
                self._warned_frames.add(incoming)
                self.get_logger().warn(
                    f"Scan frame_id '{incoming}' not in TF tree; rewriting to {self.output_frame}"
                )
            msg.header.frame_id = self.output_frame

        # Keep original stamp so TF lookup uses correct time (no "speed circles").

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