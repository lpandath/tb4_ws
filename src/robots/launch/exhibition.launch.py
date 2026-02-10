#!/usr/bin/env python3
"""
Launch master exhibition controller with movement mode selection.

Usage:
  ros2 launch robots exhibition.launch.py movement_mode:=swing_only
  ros2 launch robots exhibition.launch.py movement_mode:=circular_only
  ros2 launch robots exhibition.launch.py movement_mode:=full_sequence
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for master exhibition controller."""

    movement_mode_arg = DeclareLaunchArgument(
        'movement_mode',
        default_value='full_sequence',
        description='Movement mode: swing_only, circular_only, or full_sequence'
    )

    robots_pkg = FindPackageShare("robots")
    config_dir = PathJoinSubstitution([robots_pkg, "config"])
    exhibition_config = PathJoinSubstitution([config_dir, "exhibition.yaml"])

    master_controller_node = Node(
        package="robots",
        executable="exhibition_master_controller",
        name="exhibition_master_controller",
        parameters=[
            exhibition_config,
            {'movement_mode': LaunchConfiguration('movement_mode')}
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        movement_mode_arg,
        master_controller_node,
    ])