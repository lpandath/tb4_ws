#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Basin robot sensor preprocessing.

    1. LaserScan frame corrector: Converts frame_id from 'rplidar_link' to 'Basin/rplidar_link'
    2. Odom to TF broadcaster: Subscribes to /Basin/odom and publishes Basin/odom -> Basin/base_link TF

    Note: The TurtleBot4 robot already publishes odometry to /Basin/odom (with namespace),
    so no relay is needed. We just need to convert odom messages to TF transforms.
    """

    # Launch-time delay to allow AMCL to bring up map->odom before scans arrive
    from launch.actions import DeclareLaunchArgument, TimerAction
    from launch.substitutions import LaunchConfiguration

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
        namespace="Basin",
        parameters=[
            {
                "namespace": "Basin",
                "input_topic": "scan",
                "output_topic": "scan_new",
                "input_frame": "rplidar_link",
                "output_frame": "rplidar_link",
            }
        ],
    )

    odom_to_tf_broadcaster = Node(
        package="robots",
        executable="odom_to_tf_broadcaster",
        name="odom_to_tf_broadcaster",
        output="screen",
        namespace="Basin",
        remappings=[("odom", "odom")],
    )

    delayed_group = TimerAction(period=startup_delay, actions=[scan_frame_corrector, odom_to_tf_broadcaster])

    return LaunchDescription(
        [
            declare_startup_delay,
            delayed_group,
        ]
    )