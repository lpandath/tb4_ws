#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch localization for Moon robot (map_server + AMCL only).

    Usage:
        ros2 launch robots moon_localization.launch.py
    """

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="Moon", description="Robot namespace"
    )

    # Get config directory from robots package
    robots_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(robots_share, "config", "moon_localization.yaml")
    default_map_file = os.path.join(robots_share, "maps", "moon_map.yaml")

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
            localization_group,
        ]
    )