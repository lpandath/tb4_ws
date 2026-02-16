#!/usr/bin/env python3
"""
Phase 1 Launch: Exhibition dance with 360° safety stop.
Manual start (no duty cycle, no auto-start).

  ros2 launch robots phase1.launch.py robots:=Moon
  ros2 launch robots phase1.launch.py robots:=Moon,Basin speed:=0.04
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    robots = context.perform_substitution(LaunchConfiguration("robots"))
    speed = float(context.perform_substitution(LaunchConfiguration("speed")))

    return [
        Node(
            package="robots",
            executable="phase1_controller",
            name="phase1_controller",
            output="screen",
            parameters=[
                {"robots": robots},
                {"scan_topic": "scan"},
                {"rotation_angle": 3.14},
                {"max_angular_speed": speed},
                {"min_angular_speed": speed * 0.5},
                {"max_accel_angular": 0.02},
                {"control_rate": 25.0},
                {"close_stop": 0.80},
                {"close_resume": 0.90},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="Moon",
                              description="Moon, Basin, or Moon,Basin"),
        DeclareLaunchArgument("speed", default_value="0.08",
                              description="Max angular speed (rad/s)"),
        OpaqueFunction(function=_launch),
    ])
