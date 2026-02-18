#!/usr/bin/env python3
"""
Phase 1 FULL Launch — one command for exhibition.
Continuous full rotation with duty cycle.

  ros2 launch robots phase1_full.launch.py robots:=Moon,Basin
  ros2 launch robots phase1_full.launch.py robots:=Moon
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    robots_str = context.perform_substitution(LaunchConfiguration("robots"))
    speed = float(context.perform_substitution(LaunchConfiguration("speed")))
    angle = float(context.perform_substitution(LaunchConfiguration("rotation_angle")))
    duty = context.perform_substitution(LaunchConfiguration("duty_cycle")).lower() == "true"

    return [
        Node(
            package="robots",
            executable="phase1_controller",
            name="phase1_controller",
            output="screen",
            parameters=[
                {"robots": robots_str},
                {"scan_topic": "scan"},
                {"rotation_angle": angle},
                {"max_angular_speed": speed},
                {"min_angular_speed": speed * 0.5},
                {"max_accel_angular": 0.02},
                {"control_rate": 25.0},
                {"close_stop": 0.80},
                {"close_resume": 0.90},
                {"auto_start": True},
                {"duty_cycle": duty},
                {"active_duration": 90.0},
                {"rest_duration": 480.0},
                {"continuous_rotation": True},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="Moon,Basin",
                              description="Moon, Basin, or Moon,Basin"),
        DeclareLaunchArgument("speed", default_value="0.08",
                              description="Max angular speed (rad/s)"),
        DeclareLaunchArgument("rotation_angle", default_value="3.14",
                              description="Oscillation sweep angle in radians (3.14=180deg)"),
        DeclareLaunchArgument("duty_cycle", default_value="true",
                              description="Enable duty cycle (true/false)"),
        OpaqueFunction(function=_launch),
    ])
