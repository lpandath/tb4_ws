#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	"""
	TF Relay for Basin and Moon: Republishes namespaced TF topics to global /tf.

	This allows RViz and other tools to see all robots' transforms in one TF tree.
	Frames are prefixed with namespace (e.g., "Basin/map", "Moon/map").
	"""

	tf_relay = Node(
		package="robots",
		executable="tf_relay",
		arguments=["Moon"],
	)

	return LaunchDescription(
		[
			tf_relay,
		]
	)
