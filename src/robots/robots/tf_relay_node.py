#!/usr/bin/env python3

import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self, namespace: str):
        ns_clean = namespace.lstrip("/")
        super().__init__("tf_relay_" + ns_clean)
        self.frame_prefix = ns_clean + "/"

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        tf_topic = "/" + ns_clean + "/tf"
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=qos_profile,
        )
        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf",
            qos_profile=qos_profile,
        )

    def tf_callback(self, msg: TFMessage) -> None:
        # Use laptop's current time to avoid TF_OLD_DATA errors from clock drift
        current_time = self.get_clock().now().to_msg()
        for transform in msg.transforms:
            # Re-timestamp to use laptop time
            transform.header.stamp = current_time
            if not transform.header.frame_id.startswith(self.frame_prefix):
                transform.header.frame_id = (
                    self.frame_prefix + transform.header.frame_id
                )
            if not transform.child_frame_id.startswith(self.frame_prefix):
                transform.child_frame_id = self.frame_prefix + transform.child_frame_id

        self.publisher.publish(msg)


class TFStaticRelay(Node):
    def __init__(self, namespace: str):
        ns_clean = namespace.lstrip("/")
        super().__init__("tf_static_relay_" + ns_clean)
        self.frame_prefix = ns_clean + "/"

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        tf_static_topic = "/" + ns_clean + "/tf_static"
        self.static_subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_static_topic,
            callback=self.static_tf_callback,
            qos_profile=qos_profile,
        )

        self.static_publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf_static",
            qos_profile=qos_profile,
        )

    def static_tf_callback(self, msg: TFMessage) -> None:
        # Use laptop's current time to avoid TF_OLD_DATA errors from clock drift
        current_time = self.get_clock().now().to_msg()
        for transform in msg.transforms:
            # Re-timestamp to use laptop time
            transform.header.stamp = current_time
            if not transform.header.frame_id.startswith(self.frame_prefix):
                transform.header.frame_id = (
                    self.frame_prefix + transform.header.frame_id
                )
            if not transform.child_frame_id.startswith(self.frame_prefix):
                transform.child_frame_id = self.frame_prefix + transform.child_frame_id

        self.static_publisher.publish(msg)


def _parse_namespaces(argv) -> list[str]:
    if len(argv) <= 1:
        return ["Basin", "Moon"]

    filtered_args = [
        arg for arg in argv[1:] if not arg.startswith("-") and ":=" not in arg
    ]

    if filtered_args:
        return [str(ns).lstrip("/") for ns in filtered_args]

    return ["Basin", "Moon"]


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()

    namespaces = _parse_namespaces(sys.argv)

    tf_relays: list[TFRelay] = []
    tf_static_relays: list[TFStaticRelay] = []

    try:
        for namespace in namespaces:
            tf_relay = TFRelay(namespace=namespace)
            tf_static_relay = TFStaticRelay(namespace=namespace)
            tf_relays.append(tf_relay)
            tf_static_relays.append(tf_static_relay)

        for relay in tf_relays:
            executor.add_node(relay)

        for relay in tf_static_relays:
            executor.add_node(relay)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        executor.shutdown()
    finally:
        for relay in tf_relays:
            relay.destroy_node()
        for relay in tf_static_relays:
            relay.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
