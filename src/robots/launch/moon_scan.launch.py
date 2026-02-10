#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Moon robot sensor preprocessing.

    1. LaserScan frame corrector: Converts frame_id from 'rplidar_link' to 'Moon/rplidar_link'
    2. Odom to TF broadcaster: Subscribes to /Moon/odom and publishes Moon/odom -> Moon/base_link TF

    Note: The TurtleBot4 robot already publishes odometry to /Moon/odom (with namespace),
    so no relay is needed. We just need to convert odom messages to TF transforms.
    """

    # Launch-time delay to allow AMCL to bring up map->odom before scans arrive
    startup_delay = LaunchConfiguration("startup_delay")

    declare_startup_delay = DeclareLaunchArgument(
        "startup_delay",
        default_value="2.0",
        description="Seconds to wait before starting scan + odom->tf nodes",
    )

    scan_frame_corrector = Node(
        package="robots",
        executable="laserscan_frame_corrector",
        name="laserscan_frame_corrector",
        output="screen",
        namespace="Moon",
        parameters=[
            {
                "namespace": "Moon",
                "input_topic": "scan",
                "output_topic": "scan_new",
                "input_frame": "rplidar_link",
                "output_frame": "rplidar_link",
            }
        ],
    )

    # Odom to TF broadcaster: converts /Moon/odom messages to TF (Moon/odom -> Moon/base_link)
    # The robot already publishes to /Moon/odom with RELIABLE + TRANSIENT_LOCAL QoS
    odom_to_tf_broadcaster = Node(
        package="robots",
        executable="odom_to_tf_broadcaster",
        name="odom_to_tf_broadcaster",
        output="screen",
        namespace="Moon",
        remappings=[("odom", "odom")],  # Subscribes to /Moon/odom
    )

    # Wait `startup_delay` seconds before launching the scan/TF nodes so AMCL
    # and map_server have time to publish `Moon/map -> Moon/odom`.
    delayed_group = TimerAction(period=startup_delay, actions=[scan_frame_corrector, odom_to_tf_broadcaster])

    return LaunchDescription(
        [
            declare_startup_delay,
            delayed_group,
        ]
    )