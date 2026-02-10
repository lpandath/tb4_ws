#!/usr/bin/env python3
"""
Launch Phase 2 circle controller.
Requires: both robots' localization running so map_frame -> robot/base_link is available on /tf.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    pkg_share = get_package_share_path("robots")
    config_path = os.path.join(pkg_share, "config", "phase2_circle.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=config_path,
            description="Path to phase2_circle.yaml",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="Moon/map",
            description="Map frame id (same for both robots)",
        ),
        DeclareLaunchArgument(
            "robots",
            default_value="Moon",
            description="Comma-separated: Moon or Moon,Basin",
        ),
        DeclareLaunchArgument(
            "circle_center_x",
            default_value="0.0",
            description="Circle center x in map (m)",
        ),
        DeclareLaunchArgument(
            "circle_center_y",
            default_value="0.0",
            description="Circle center y in map (m)",
        ),
        DeclareLaunchArgument(
            "circle_radius",
            default_value="1.0",
            description="Circle radius (m)",
        ),
        DeclareLaunchArgument(
            "circle_direction",
            default_value="1",
            description="1=CCW, -1=CW",
        ),
        DeclareLaunchArgument(
            "linear_speed",
            default_value="0.15",
            description="Tangent speed (m/s)",
        ),
        DeclareLaunchArgument(
            "start_immediately",
            default_value="false",
            description="Start circling on launch",
        ),
        Node(
            package="robots",
            executable="phase2_circle_controller",
            name="phase2_circle_controller",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {"map_frame": LaunchConfiguration("map_frame")},
                {"robots": LaunchConfiguration("robots")},
                {"circle_center_x": LaunchConfiguration("circle_center_x")},
                {"circle_center_y": LaunchConfiguration("circle_center_y")},
                {"circle_radius": LaunchConfiguration("circle_radius")},
                {"circle_direction": LaunchConfiguration("circle_direction")},
                {"linear_speed": LaunchConfiguration("linear_speed")},
                {"start_immediately": LaunchConfiguration("start_immediately")},
            ],
        ),
    ])