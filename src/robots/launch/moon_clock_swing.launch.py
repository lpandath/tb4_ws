#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory("robots"),
        "config",
        "moon_clock_swing.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="robots",
                executable="clock_swing_controller",
                name="clock_swing_controller",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
