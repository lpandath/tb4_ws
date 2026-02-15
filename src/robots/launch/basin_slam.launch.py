#!/usr/bin/env python3
# basin_slam.launch.py - LifecycleNode with auto-configure/activate

import os

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robots_share = get_package_share_directory("robots")
    params_file = os.path.join(robots_share, "config", "basin_slam.yaml")

    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="Basin",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": False,
                "scan_topic": "scan_new",
                "map_frame": "Basin/map",
                "odom_frame": "Basin/odom",
                "base_frame": "Basin/base_link",
                "transform_tolerance": 10.0,
                "minimum_time_interval": 2.0,
                "throttle_scans": 5,
                "do_loop_closing": False,
                "ceres_max_num_iterations": 5,
                "debug_logging": True,
                "qos_scan": 0,
            },
        ],
        remappings=[
            ("scan", "scan_new"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("map", "/map"),
            ("map_metadata", "/map_metadata"),
        ],
    )

    # Auto-configure the node on launch
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate after configure succeeds
    activate_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        slam_toolbox_node,
        configure_event,
        activate_handler,
    ])
