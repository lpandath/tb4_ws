#!/usr/bin/env python3
"""
Launch clock swing controller with Moon and Basin robots.
Configurable motion profiles via YAML.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for clock swing controller."""

    # Path to robots package configuration
    robots_pkg = FindPackageShare("robots")
    config_dir = PathJoinSubstitution([robots_pkg, "config"])
    clock_config = PathJoinSubstitution([config_dir, "clock_swing.yaml"])

    # Clock swing controller node
    clock_swing_node = Node(
        package="robots",
        executable="clock_swing_controller",
        name="clock_swing_controller",
        parameters=[clock_config],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        clock_swing_node,
    ])
