#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch localization for Basin robot.

    Includes basin_scan (frame corrector + odom->base_link TF) so scan_new and
    TF are produced on the same machine as AMCL. That avoids "timestamp on the
    message is earlier than all the data in the transform cache" from clock
    skew when scan and TF come from different machines.

    Usage:
        ros2 launch robots basin_localization.launch.py
    """

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock (false for real robot)"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="Basin", description="Robot namespace"
    )

    # Get config directory from robots package
    robots_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(
        robots_share, "config", "basin_localization.yaml"
    )
    default_map_file = os.path.join(robots_share, "maps", "basin_map.yaml")
    basin_scan_launch = os.path.join(robots_share, "launch", "basin_scan.launch.py")

    declare_map_file = DeclareLaunchArgument(
        "map_file",
        default_value=default_map_file,
        description="Path to the map YAML file",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_yaml,
        description="Path to AMCL params YAML",
    )

    # Map server - loads saved map (inside namespace)
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_file},
            {"topic_name": "map"},
            {"frame_id": PathJoinSubstitution([namespace, "map"])},
            {"use_sim_time": use_sim_time},
        ],
    )

    # AMCL node: remap initialpose to /initialpose so RViz "2D Pose Estimate" works
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("scan", "scan_new"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("initialpose", "/initialpose"),
        ],
    )

    # Lifecycle manager for both nodes
    lifecycle_nodes = ["map_server", "amcl"]

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": lifecycle_nodes},
            {"bond_timeout": 60.0},
        ],
    )

    # Scan pipeline (frame corrector + odom->base_link TF). Run on same machine
    # as AMCL so scan_new and TF share the same clock and message filter does
    # not drop scans ("timestamp earlier than transform cache").
    include_basin_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(basin_scan_launch),
    )

    # Group ALL nodes with namespace
    localization_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            map_server,
            amcl_node,
            lifecycle_manager,
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_namespace,
            declare_map_file,
            declare_params_file,
            include_basin_scan,
            localization_group,
        ]
    )