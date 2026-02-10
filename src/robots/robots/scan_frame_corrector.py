#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameCorrector(Node):
    def __init__(self):
        super().__init__('scan_frame_corrector')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Subscribes to /Moon/scan (with namespace)
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            LaserScan,
            'scan_corrected',  # Publishes to /Moon/scan_corrected
            10)
        self.get_logger().info('ScanFrameCorrector node started.')

    def listener_callback(self, msg):
        # Change frame_id to Moon/rplidar_link
        msg.header.frame_id = f'{self.get_namespace().strip('/')}/rplidar_link'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameCorrector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
