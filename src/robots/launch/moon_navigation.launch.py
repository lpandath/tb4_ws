#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch navigation only for Moon robot.
    Run localization separately first (moon_localization.launch.py), set /Moon/initialpose, then run this.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="Moon", description="Robot namespace"
    )

    moon_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(moon_share, "config", "moon_navigation.yaml")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_yaml,
        description="Path to nav2 params YAML",
    )

    # Note: Moon/base_link -> Moon/rplidar_link is already published by the robot
    # No need to add static transform here

    # Controller server - publishes to cmd_vel_unstamped (what TurtleBot4 listens to)
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cmd_vel", "cmd_vel_unstamped"),  # TurtleBot4 listens here
            ("odom", "odom"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("local_costmap/costmap_raw", "local_costmap/costmap_raw"),
            ("local_costmap/costmap_updates", "local_costmap/costmap_updates"),
        ],
    )

    # Planner server - NEEDS MAP TOPIC
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("map", "map"),  # /Moon/map
        ],
    )

    # Behavior server
    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("local_costmap/costmap_raw", "local_costmap/costmap_raw"),
            ("local_costmap/published_footprint", "local_costmap/published_footprint"),
        ],
    )

    # BT Navigator - uses GLOBAL topics for RViz compatibility
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("odom", "odom"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("initialpose", "/initialpose"),  # GLOBAL - RViz publishes here
            ("goal_pose", "/goal_pose"),  # GLOBAL - RViz 2D Goal Pose publishes here
            ("navigate_to_pose", "navigate_to_pose"),  # /Moon/navigate_to_pose
            (
                "navigate_through_poses",
                "navigate_through_poses",
            ),  # /Moon/navigate_through_poses
        ],
    )

    # Waypoint follower
    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    # Velocity smoother
    # Input: cmd_vel_nav (from controller_server)
    # Output: cmd_vel_unstamped (to robot)
    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),  # Input from controller
            ("cmd_vel_smoothed", "cmd_vel_unstamped"),  # Output to robot
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("odom", "odom"),
        ],
    )

    # Lifecycle manager - removed velocity_smoother (TurtleBot4 has built-in smoothing)
    nav_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        # "velocity_smoother",  # Disabled - TurtleBot4 has its own
    ]

    nav_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": nav_lifecycle_nodes},
            {"bond_timeout": 30.0},
        ],
    )

    # Group all nodes with namespace
    # Note: odom_to_tf_broadcaster is launched separately in moon_scan.launch.py
    # Note: velocity_smoother removed - TurtleBot4 has built-in velocity handling
    nav_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            # velocity_smoother_node,  # Disabled
            nav_lifecycle_manager,
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_namespace,
            declare_params_file,
            nav_group,
        ]
    )
#