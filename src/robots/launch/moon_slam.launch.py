#!/usr/bin/env python3
# moon_slam_sync.launch.py - FIXED VERSION

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robots_share = get_package_share_directory("robots")
    params_file = os.path.join(robots_share, "config", "moon_slam.yaml")
    
    # Sync version - no lifecycle, starts immediately
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="Moon",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": False,
                "scan_topic": "scan_new",
                "map_frame": "Moon/map",
                "odom_frame": "Moon/odom", 
                "base_frame": "Moon/base_link",
                "transform_tolerance": 10.0,
                "minimum_time_interval": 2.0,
                "throttle_scans": 5,
                "do_loop_closing": False,
                "ceres_max_num_iterations": 5,  # FIXED: correct parameter name
                "debug_logging": True,
                "qos_scan": 0,  # 0=RELIABLE, 1=BEST_EFFORT
            }
        ],
        remappings=[
            ("scan", "scan_new"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("map", "/map"),
            ("map_metadata", "/map_metadata"),
        ],
    )

    return LaunchDescription([
        slam_toolbox_node,
    ])