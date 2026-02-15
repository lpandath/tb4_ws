#!/usr/bin/env python3
"""
Phase 1 FULL Launch — one command for exhibition.
Subscribes to /scan directly (no scan correctors needed). Auto-starts.

  ros2 launch robots phase1_full.launch.py robots:=Moon
  ros2 launch robots phase1_full.launch.py robots:=Moon,Basin
  ros2 launch robots phase1_full.launch.py robots:=Basin duty_cycle:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    robots_str = context.perform_substitution(LaunchConfiguration("robots"))
    speed = float(context.perform_substitution(LaunchConfiguration("speed")))
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
                {"rotation_angle": 2.79},
                {"max_angular_speed": speed},
                {"min_angular_speed": speed * 0.5},
                {"max_accel_angular": 0.04},
                {"control_rate": 25.0},
                {"close_stop": 0.80},
                {"close_resume": 0.90},
                {"auto_start": True},
                {"duty_cycle": duty},
                {"active_duration": 90.0},
                {"rest_duration": 420.0},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="Moon,Basin",
                              description="Moon, Basin, or Moon,Basin"),
        DeclareLaunchArgument("speed", default_value="0.06",
                              description="Max angular speed (rad/s)"),
        DeclareLaunchArgument("duty_cycle", default_value="false",
                              description="Enable duty cycle (true/false)"),
        OpaqueFunction(function=_launch),
    ])
