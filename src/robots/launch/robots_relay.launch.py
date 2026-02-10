#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    TF Relay for both robots (Moon + Basin).

    Republishes namespaced TF topics to global `/tf` so RViz/Foxglove
    can see both robots in one TF tree.
    """

    # Run a single tf_relay process that handles both namespaces to avoid
    # duplicate logger/publisher registration warnings from multiple processes.
    tf_relay = Node(
        package="robots",
        executable="tf_relay",
        name="tf_relay",
        output="screen",
        arguments=["Moon", "Basin"],
    )

    return LaunchDescription([
        tf_relay,
    ])
