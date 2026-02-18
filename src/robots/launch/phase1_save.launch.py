#!/usr/bin/env python3
"""
Phase 1 POWER-SAVE Launch — longer rest periods for battery conservation.
Subscribes to /scan directly (no scan correctors needed). Auto-starts.

  ros2 launch robots phase1_save.launch.py robots:=Moon
  ros2 launch robots phase1_save.launch.py robots:=Moon,Basin
  ros2 launch robots phase1_save.launch.py robots:=Basin duty_cycle:=true
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
                {"max_accel_angular": 0.04},
                {"control_rate": 25.0},
                {"close_stop": 0.80},
                {"close_resume": 0.90},
                {"auto_start": True},
                {"duty_cycle": duty},
                {"active_duration": 60.0},
                {"rest_duration": 720.0},
                {"stop_on_scan_loss": False},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="Moon,Basin",
                              description="Moon, Basin, or Moon,Basin"),
        DeclareLaunchArgument("speed", default_value="0.04",
                              description="Max angular speed (rad/s)"),
        DeclareLaunchArgument("rotation_angle", default_value="2.79",
                              description="Sweep angle in radians (2.79=160deg, 1.57=90deg)"),
        DeclareLaunchArgument("duty_cycle", default_value="true",
                              description="Enable duty cycle (true/false)"),
        OpaqueFunction(function=_launch),
    ])
