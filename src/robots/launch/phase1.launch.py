#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Audience zone: move when in range (stop–far), idle when too close or > far (+ hysteresis)
DISTANCES_HOME = {  # small room: stop <25cm (testing), move 25–80cm, idle >80cm
    "distance_stop": 0.25,
    "distance_far": 0.80,
    "far_hysteresis": 0.0,  # idle as soon as > 80cm (no extra band)
}
DISTANCES_EXHIBITION = {  # large room: move 20cm–1m, idle when > 1m + hysteresis
    "distance_stop": 0.20,
    "distance_far": 1.0,
    "far_hysteresis": 0.25,
}


def _launch_phase1_controller(context, *args, **kwargs):
    env = context.perform_substitution(LaunchConfiguration("env"))
    robots_val = context.perform_substitution(LaunchConfiguration("robots"))
    speed_val = float(context.perform_substitution(LaunchConfiguration("speed")))
    dist = DISTANCES_EXHIBITION if env == "exhibition" else DISTANCES_HOME
    node = Node(
        package="robots",
        executable="phase1_controller",
        name="phase1_controller",
        output="screen",
        parameters=[
            {"robots": robots_val},
            {"linear_speed": 0.15},
            {"target_radius": 0.3},
            {"rotation_angle": 3.141592653589793},  # π rad = 180° semicircle
            {"distance_stop": dist["distance_stop"]},
            {"distance_far": dist["distance_far"]},
            {"far_hysteresis": dist["far_hysteresis"]},
            {"min_angular_speed": speed_val},
            {"max_angular_speed": speed_val},
            {"half_circle_duration": 300.0},
            {"total_duration": 600.0},
            {"control_rate": 30.0},
            {"auto_start": False},
        ],
    )
    return [node]


def generate_launch_description():
    """
    Phase 1: Semicircle back-and-forth (annual clock). Moon/Basin mirrored.

    Launch args:
      robots:=Moon | Basin | Moon,Basin
      env:=home       (default) move 25–80cm, idle when close or far
      env:=exhibition move when 20cm–1m
      speed:=0.05     (default) rotation speed in rad/s; higher = faster (e.g. 0.08), lower = slower (e.g. 0.03)
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots",
                default_value="Moon",
                description="Moon, Basin, or Moon,Basin",
            ),
            DeclareLaunchArgument(
                "env",
                default_value="home",
                description="home (small room) or exhibition (larger distances); both robots use same preset",
            ),
            DeclareLaunchArgument(
                "speed",
                default_value="0.05",
                description="Rotation speed in rad/s (e.g. 0.05 gentle, 0.08 faster)",
            ),
            OpaqueFunction(function=_launch_phase1_controller),
        ]
    )