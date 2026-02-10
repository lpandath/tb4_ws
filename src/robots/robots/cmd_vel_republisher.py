#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class CmdVelRepublisher(Node):
    """Subscribe to a raw `Twist` topic and republish as `TwistStamped`.

    Useful when robot driver expects `TwistStamped` but controllers/teleop
    publish plain `Twist` (common on TurtleBot4 setups).
    """

    def __init__(self):
        super().__init__("cmd_vel_republisher")

        # Namespace from node namespace
        self.namespace = self.get_namespace().strip("/")

        # Parameters
        self.declare_parameter("input_topic", "cmd_vel_unstamped")
        self.declare_parameter("output_topic", "cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        frame_id = self.get_parameter("frame_id").value

        # QoS: best-effort for cmd_vel
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        self.pub = self.create_publisher(TwistStamped, output_topic, qos)
        self.sub = self.create_subscription(Twist, input_topic, self.cb_twist, qos)

        self.get_logger().info(
            f"CmdVelRepublisher started in namespace='{self.namespace}': {input_topic} -> {output_topic} (frame: {frame_id})"
        )
        self.frame_id = f"{self.namespace}/{frame_id}" if self.namespace else frame_id

    def cb_twist(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.frame_id
        ts.twist = msg
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
