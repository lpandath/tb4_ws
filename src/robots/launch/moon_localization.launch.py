#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

NS = "Moon"


def generate_launch_description():
    """
    Launch localization for Moon robot.

    Uses namespace= directly on each node (not PushRosNamespace) so that
    YAML parameter files with 'Moon: node_name: ros__parameters:' format
    load correctly.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("localization_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
    )

    robots_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(robots_share, "config", "moon_localization.yaml")
    default_map_file = os.path.join(robots_share, "maps", "map_small_room.yaml")
    moon_scan_launch = os.path.join(robots_share, "launch", "moon_scan.launch.py")

    declare_map_file = DeclareLaunchArgument(
        "map_file", default_value=default_map_file,
    )
    declare_params_file = DeclareLaunchArgument(
        "localization_params_file", default_value=default_params_yaml,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace=NS,
        output="screen",
        parameters=[
            {"yaml_filename": map_file},
            {"topic_name": "map"},
            {"frame_id": f"{NS}/map"},
            {"use_sim_time": use_sim_time},
        ],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("scan", "scan_new"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("initialpose", "/initialpose"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=NS,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
            {"bond_timeout": 60.0},
        ],
    )

    include_moon_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moon_scan_launch),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,
        declare_params_file,
        include_moon_scan,
        map_server,
        amcl_node,
        lifecycle_manager,
    ])
