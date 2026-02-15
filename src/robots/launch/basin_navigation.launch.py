#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

NS = "Basin"


def generate_launch_description():
    """
    Launch navigation only for Basin robot.

    Run localization separately first (basin_localization.launch.py),
    set initial pose, then run this.

    Uses namespace= directly on each node (not PushRosNamespace) so that
    YAML parameter files with 'Basin: node_name: ros__parameters:' format
    load correctly.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
    )

    robots_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(robots_share, "config", "basin_navigation.yaml")

    declare_params_file = DeclareLaunchArgument(
        "params_file", default_value=default_params_yaml,
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cmd_vel", "cmd_vel_unstamped"),
            ("odom", "odom"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("odom", "odom"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("initialpose", "/initialpose"),
            ("goal_pose", "/goal_pose"),
        ],
    )

    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=NS,
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    nav_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=NS,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
            ]},
            {"bond_timeout": 30.0},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        nav_lifecycle_manager,
    ])
