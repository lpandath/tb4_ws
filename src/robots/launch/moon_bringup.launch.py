#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Single launch for Moon: scan → relay → localisation (event-based).

    Starts relay only after odom data is received, and localisation only after
    scan data is received (so the laser has time to publish and AMCL gets data).
    Optional timeout if data never appears. Navigation/waypoints not included;
    run moon_navigation.launch.py when needed.

    Usage:
        ros2 launch robots moon_bringup.launch.py
    """
    robots_share = get_package_share_directory("robots")
    launch_dir = os.path.join(robots_share, "launch")

    delay_scan_start = LaunchConfiguration("delay_scan_start", default="0.0")
    wait_timeout = LaunchConfiguration("wait_timeout", default="30.0")

    declare_delay_scan_start = DeclareLaunchArgument(
        "delay_scan_start",
        default_value="0.0",
        description="Seconds before starting scan nodes (0 = immediate).",
    )
    declare_wait_timeout = DeclareLaunchArgument(
        "wait_timeout",
        default_value="30.0",
        description="Timeout (s) waiting for odom/scan data before giving up.",
    )

    # 1) Moon scan (frame corrector + odom→TF). Start first.
    moon_scan_launch = os.path.join(launch_dir, "moon_scan.launch.py")
    include_moon_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moon_scan_launch),
        launch_arguments=[("startup_delay", delay_scan_start)],
    )

    # 2) Wait for odom: when we receive first /Moon/odom, start relay and wait_for_scan.
    wait_for_odom = Node(
        package="robots",
        executable="wait_for_topic",
        name="wait_for_odom",
        output="screen",
        parameters=[
            {"topic": "/Moon/odom"},
            {"message_type": "Odometry"},
            {"timeout": wait_timeout},
        ],
    )

    moon_relay_launch = os.path.join(launch_dir, "moon_relay.launch.py")
    include_moon_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moon_relay_launch),
    )

    # 3) Wait for scan_new: when we receive first /Moon/scan_new, start localisation.
    wait_for_scan = Node(
        package="robots",
        executable="wait_for_topic",
        name="wait_for_scan",
        output="screen",
        parameters=[
            {"topic": "/Moon/scan_new"},
            {"message_type": "LaserScan"},
            {"timeout": wait_timeout},
        ],
    )

    moon_localization_launch = os.path.join(launch_dir, "moon_localization.launch.py")
    include_moon_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moon_localization_launch),
    )

    # When odom is received: start relay and start wait_for_scan; when that exits, start localisation.
    on_odom_received = [
        include_moon_relay,
        wait_for_scan,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_scan,
                on_exit=[include_moon_localization],
            )
        ),
    ]

    return LaunchDescription(
        [
            declare_delay_scan_start,
            declare_wait_timeout,
            include_moon_scan,
            wait_for_odom,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wait_for_odom,
                    on_exit=on_odom_received,
                )
            ),
        ]
    )
