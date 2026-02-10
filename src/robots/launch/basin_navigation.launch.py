#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch navigation for Basin robot.

    Includes basin_localization (map_server + AMCL + scan pipeline) so the
    global costmap receives the map on /Basin/map. Without it you get
    "Can't update static costmap layer, no map received" and
    "Robot is out of bounds of the costmap".
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock (false for real robot)"
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="Basin", description="Robot namespace"
    )

    robots_share = get_package_share_directory("robots")
    default_params_yaml = os.path.join(robots_share, "config", "basin_navigation.yaml")
    basin_localization_launch = os.path.join(
        robots_share, "launch", "basin_localization.launch.py"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_yaml,
        description="Path to nav2 params YAML",
    )

    # Controller server
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("cmd_vel", "cmd_vel"),
            ("odom", "odom"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("local_costmap/costmap_raw", "local_costmap/costmap_raw"),
            ("local_costmap/costmap_updates", "local_costmap/costmap_updates"),
        ],
    )

    # Localization (map_server + AMCL + scan). Must run first so /Basin/map
    # is published before planner_server's global costmap starts.
    include_basin_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(basin_localization_launch),
    )

    # Planner server - subscribes to map (resolved to /Basin/map)
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("map", "map"),  # /Basin/map
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

    # BT Navigator - uses GLOBAL topics for RViz/Foxglove compatibility
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
            ("navigate_to_pose", "navigate_to_pose"),  # /Basin/navigate_to_pose
            (
                "navigate_through_poses",
                "navigate_through_poses",
            ),  # /Basin/navigate_through_poses
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
            include_basin_localization,
            nav_group,
        ]
    )