#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Robust bringup for Moon robot:
    1. Start scan pipeline (frame corrector, odom->tf broadcaster)
    2. Wait (delay) for TFs to fill
    3. Start localization (map_server + AMCL)
    4. Wait (delay) for map->odom to appear
    5. Start navigation
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_delay = LaunchConfiguration("scan_delay")
    localization_delay = LaunchConfiguration("localization_delay")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="Moon", description="Robot namespace"
    )
    declare_scan_delay = DeclareLaunchArgument(
        "scan_delay", default_value="2.0", description="Delay before starting localization (sec)"
    )
    declare_localization_delay = DeclareLaunchArgument(
        "localization_delay", default_value="3.0", description="Delay before starting navigation (sec)"
    )

    robots_share = get_package_share_directory("robots")

    scan_launch = os.path.join(robots_share, "launch", "moon_scan.launch.py")
    localization_launch = os.path.join(robots_share, "launch", "moon_localization.launch.py")
    navigation_launch = os.path.join(robots_share, "launch", "moon_navigation.launch.py")

    scan_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scan_launch),
        launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
    )
    delayed_localization = TimerAction(
        period=scan_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localization_launch),
                launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
            )
        ],
    )
    delayed_navigation = TimerAction(
        period=localization_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
            )
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_scan_delay,
        declare_localization_delay,
        scan_group,
        delayed_localization,
        delayed_navigation,
    ])
