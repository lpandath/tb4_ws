#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    moon_scan = DeclareLaunchArgument("moon_scan", default_value="scan_new")
    basin_scan = DeclareLaunchArgument("basin_scan", default_value="scan")
    run_duration = DeclareLaunchArgument("run_duration_s", default_value="600")
    shutdown_on_finish = DeclareLaunchArgument("shutdown_on_finish", default_value="false")
    start_immediately = DeclareLaunchArgument("start_immediately", default_value="false")
    run_duration_from_launch = DeclareLaunchArgument("run_duration_from_launch", default_value="false")
    angular_speed = DeclareLaunchArgument("angular_speed", default_value="0.30")
    sweep_angle_rad = DeclareLaunchArgument("sweep_angle_rad", default_value="6.283185307179586")
    reverse_each_cycle = DeclareLaunchArgument("reverse_each_cycle", default_value="false")
    max_angular_accel = DeclareLaunchArgument("max_angular_accel", default_value="0.8")

    # Defaults tuned for an open exhibition space:
    # - Start when a person is within ~1.2m
    # - Pause if they get closer than ~0.6m
    # - Resume once they step back past ~0.9m
    common_params = [
        {"mode": "in_place"},
        {"sweep_angle_rad": LaunchConfiguration("sweep_angle_rad")},
        {"reverse_each_cycle": LaunchConfiguration("reverse_each_cycle")},
        {"radius": 0.8},
        {"linear_speed": 0.0},
        # Slower/quieter rotation.
        {"angular_speed": LaunchConfiguration("angular_speed")},
        {"stop_distance": 0.60},
        {"resume_distance": 0.90},
        {"start_distance": 1.20},
        # Wider cone is OK in open space.
        {"scan_angle_min_deg": -75.0},
        {"scan_angle_max_deg": 75.0},
        {"scan_min_range": 0.2},
        # Robust gating: require a small cluster of close beams.
        {"min_k_for_distance": 8},
        {"stop_beams_required": 2},
        {"start_beams_required": 2},
        # Debounce to avoid rapid toggling.
        {"debounce_start_s": 0.20},
        {"debounce_idle_s": 0.30},
        {"debounce_pause_s": 0.10},
        {"debounce_resume_s": 0.20},
        {"control_rate": 20.0},
        {"max_angular_accel": LaunchConfiguration("max_angular_accel")},
        {"scan_timeout_s": 5.0},
        {"debug_log": True},
        {"auto_restart": True},
        {"run_duration_s": LaunchConfiguration("run_duration_s")},
        {"run_duration_from_launch": LaunchConfiguration("run_duration_from_launch")},
        {"shutdown_on_finish": LaunchConfiguration("shutdown_on_finish")},
        {"start_immediately": LaunchConfiguration("start_immediately")},
    ]

    moon_group = GroupAction(
        actions=[
            PushRosNamespace("Moon"),
            Node(
                package="robots",
                executable="clock_swing_single",
                name="clock_swing",
                output="screen",
                parameters=common_params
                + [
                    {"direction": -1},
                    {"scan_topic": LaunchConfiguration("moon_scan")},
                    {"cmd_vel_topics": ["cmd_vel_unstamped", "cmd_vel"]},
                ],
            ),
        ]
    )

    basin_group = GroupAction(
        actions=[
            PushRosNamespace("Basin"),
            Node(
                package="robots",
                executable="clock_swing_single",
                name="clock_swing",
                output="screen",
                parameters=common_params
                + [
                    {"direction": 1},
                    {"scan_topic": LaunchConfiguration("basin_scan")},
                    {"cmd_vel_topics": ["cmd_vel", "cmd_vel_unstamped"]},
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            moon_scan,
            basin_scan,
            run_duration,
            run_duration_from_launch,
            shutdown_on_finish,
            start_immediately,
            angular_speed,
            sweep_angle_rad,
            reverse_each_cycle,
            max_angular_accel,
            moon_group,
            basin_group,
        ]
    )
