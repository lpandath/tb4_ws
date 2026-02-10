#!/usr/bin/env python3
"""
Waypoint Recorder - Click on points in Foxglove to record waypoints.
Then press Enter to navigate through them.

Usage:
1. Run this script
2. In Foxglove 3D panel, click on the map to add waypoints
3. Watch the terminal - it will show recorded waypoints
4. Press Enter in the terminal to start navigation
5. Press 'p' + Enter to pick a circle for Phase 2 (click center, then a point on the circle)
6. Press 'c' + Enter to clear waypoints
7. Press 'q' + Enter to quit
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import threading
import sys


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")

        # Get namespace
        self.namespace = self.get_namespace().strip("/")
        if not self.namespace:
            self.namespace = "Moon"

        self.waypoints = []
        # Circle pick for Phase 2: first click = center, second = point on circle
        self.picking_circle = False
        self.circle_center = None  # (x, y) after first click
        self.circle_display = None  # (cx, cy, r) to draw on map, or None

        # Subscribe to clicked_point from Foxglove
        self.click_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.click_callback, 10
        )

        # Publisher for waypoint markers (visible in Foxglove)
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)

        # Timer to publish markers regularly so they stay visible
        self.create_timer(0.5, self.publish_markers)

        # Action client for waypoint following
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, f"/{self.namespace}/follow_waypoints"
        )

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Waypoint Recorder started for {self.namespace}")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Click on the map in Foxglove to add waypoints")
        self.get_logger().info("Commands:")
        self.get_logger().info("  [Enter] - Start navigation")
        self.get_logger().info("  [p]     - Pick circle for Phase 2 (click center, then point on circle)")
        self.get_logger().info("  [c]     - Clear waypoints")
        self.get_logger().info("  [q]     - Quit")
        self.get_logger().info("=" * 50)

        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def click_callback(self, msg: PointStamped):
        """Record clicked point as waypoint or as circle center/radius."""
        x = msg.point.x
        y = msg.point.y

        if self.picking_circle:
            if self.circle_center is None:
                self.circle_center = (x, y)
                self.get_logger().info(
                    f"Circle center: ({x:.3f}, {y:.3f}). Now click a point ON the circle (to set radius)."
                )
                return
            else:
                cx, cy = self.circle_center
                r = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                self.circle_display = (cx, cy, r)
                self.circle_center = None
                self.picking_circle = False
                self.get_logger().info("")
                self.get_logger().info("--- Phase 2 circle (from map) ---")
                self.get_logger().info(
                    f"  circle_center_x: {cx:.3f}, circle_center_y: {cy:.3f}, circle_radius: {r:.3f}"
                )
                self.get_logger().info("Run Phase 2 with:")
                self.get_logger().info(
                    f"  ros2 launch robots phase2_circle.launch.py robots:=Moon "
                    f"circle_center_x:={cx:.3f} circle_center_y:={cy:.3f} circle_radius:={r:.3f} start_immediately:=true"
                )
                self.get_logger().info("---")
                self.get_logger().info("")
                return

        self.waypoints.append((x, y))
        self.get_logger().info(
            f"Waypoint {len(self.waypoints)} recorded: x={x:.2f}, y={y:.2f}"
        )
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")

    def publish_markers(self):
        """Publish markers for waypoints and for Phase 2 circle (if picked)."""
        marker_array = MarkerArray()

        # Phase 2 circle outline (if we have center + radius from map pick)
        if self.circle_display is not None:
            cx, cy, r = self.circle_display
            circle_marker = Marker()
            circle_marker.header.frame_id = f"{self.namespace}/map"
            circle_marker.header.stamp = self.get_clock().now().to_msg()
            circle_marker.ns = "phase2_circle"
            circle_marker.id = 0
            circle_marker.type = Marker.LINE_STRIP
            circle_marker.action = Marker.ADD
            circle_marker.scale.x = 0.05
            circle_marker.color.r = 1.0
            circle_marker.color.g = 0.5
            circle_marker.color.b = 0.0
            circle_marker.color.a = 0.9
            n = 32
            for i in range(n + 1):
                th = 2.0 * math.pi * i / n
                p = Point()
                p.x = cx + r * math.cos(th)
                p.y = cy + r * math.sin(th)
                p.z = 0.05
                circle_marker.points.append(p)
            marker_array.markers.append(circle_marker)

        if not self.waypoints:
            if not marker_array.markers:
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        for i, (x, y) in enumerate(self.waypoints):
            # Sphere marker for waypoint
            marker = Marker()
            marker.header.frame_id = f"{self.namespace}/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2  # diameter
            marker.scale.y = 0.2
            marker.scale.z = 0.3  # height
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = f"{self.namespace}/map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(i + 1)

            marker_array.markers.append(text_marker)

        # Line connecting waypoints
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = f"{self.namespace}/map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "waypoint_path"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # line width
            line_marker.color.r = 0.0
            line_marker.color.g = 0.8
            line_marker.color.b = 1.0
            line_marker.color.a = 0.8

            for x, y in self.waypoints:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                line_marker.points.append(p)

            marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)

    def input_loop(self):
        """Handle keyboard input."""
        while rclpy.ok():
            try:
                user_input = input().strip().lower()

                if user_input == "q":
                    self.get_logger().info("Quitting...")
                    rclpy.shutdown()
                    break

                elif user_input == "p":
                    self.picking_circle = True
                    self.circle_center = None
                    self.get_logger().info(
                        "Pick circle for Phase 2: click the circle CENTER on the map."
                    )

                elif user_input == "c":
                    self.waypoints.clear()
                    self.picking_circle = False
                    self.circle_center = None
                    self.circle_display = None
                    self.publish_markers()  # Clear markers from display
                    self.get_logger().info("Waypoints cleared!")

                elif user_input == "" and len(self.waypoints) > 0:
                    self.get_logger().info(
                        f"Starting navigation with {len(self.waypoints)} waypoints..."
                    )
                    self.navigate()

                elif user_input == "":
                    self.get_logger().info(
                        "No waypoints recorded! Click on the map first."
                    )

            except EOFError:
                break

    def navigate(self):
        """Navigate through recorded waypoints."""
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_waypoints action server not available!")
            return

        # Create poses
        poses = []
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = f"{self.namespace}/map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        # Send goal
        goal = FollowWaypoints.Goal()
        goal.poses = poses

        future = self.waypoint_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted! Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(
            f"Navigating to waypoint {current + 1}/{len(self.waypoints)}"
        )

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints

        if len(missed) == 0:
            self.get_logger().info("All waypoints reached!")
        else:
            self.get_logger().warn(f"Missed waypoints: {list(missed)}")

        self.get_logger().info(
            "Ready for new waypoints. Click on map or press Enter to run again."
        )


def main(args=None):
    rclpy.init(args=args)#!/usr/bin/env python3
"""
Waypoint Recorder - Click on points in Foxglove to record waypoints.
Then press Enter to navigate through them.

Usage:
1. Run this script
2. In Foxglove 3D panel, click on the map to add waypoints
3. Watch the terminal - it will show recorded waypoints
4. Press Enter in the terminal to start navigation
5. Press 'p' + Enter to pick a circle for Phase 2 (click center, then a point on the circle)
6. Press 'c' + Enter to clear waypoints
7. Press 'q' + Enter to quit
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import threading
import sys


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("waypoint_recorder")

        # Get namespace
        self.namespace = self.get_namespace().strip("/")
        if not self.namespace:
            self.namespace = "Moon"

        self.waypoints = []
        # Circle pick for Phase 2: first click = center, second = point on circle
        self.picking_circle = False
        self.circle_center = None  # (x, y) after first click
        self.circle_display = None  # (cx, cy, r) to draw on map, or None

        # Subscribe to clicked_point from Foxglove
        self.click_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.click_callback, 10
        )

        # Publisher for waypoint markers (visible in Foxglove)
        self.marker_pub = self.create_publisher(MarkerArray, "/waypoint_markers", 10)

        # Timer to publish markers regularly so they stay visible
        self.create_timer(0.5, self.publish_markers)

        # Action client for waypoint following
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, f"/{self.namespace}/follow_waypoints"
        )

        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Waypoint Recorder started for {self.namespace}")
        self.get_logger().info("=" * 50)
        self.get_logger().info("Click on the map in Foxglove to add waypoints")
        self.get_logger().info("Commands:")
        self.get_logger().info("  [Enter] - Start navigation")
        self.get_logger().info("  [p]     - Pick circle for Phase 2 (click center, then point on circle)")
        self.get_logger().info("  [c]     - Clear waypoints")
        self.get_logger().info("  [q]     - Quit")
        self.get_logger().info("=" * 50)

        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def click_callback(self, msg: PointStamped):
        """Record clicked point as waypoint or as circle center/radius."""
        x = msg.point.x
        y = msg.point.y

        if self.picking_circle:
            if self.circle_center is None:
                self.circle_center = (x, y)
                self.get_logger().info(
                    f"Circle center: ({x:.3f}, {y:.3f}). Now click a point ON the circle (to set radius)."
                )
                return
            else:
                cx, cy = self.circle_center
                r = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                self.circle_display = (cx, cy, r)
                self.circle_center = None
                self.picking_circle = False
                self.get_logger().info("")
                self.get_logger().info("--- Phase 2 circle (from map) ---")
                self.get_logger().info(
                    f"  circle_center_x: {cx:.3f}, circle_center_y: {cy:.3f}, circle_radius: {r:.3f}"
                )
                self.get_logger().info("Run Phase 2 with:")
                self.get_logger().info(
                    f"  ros2 launch robots phase2_circle.launch.py robots:=Moon "
                    f"circle_center_x:={cx:.3f} circle_center_y:={cy:.3f} circle_radius:={r:.3f} start_immediately:=true"
                )
                self.get_logger().info("---")
                self.get_logger().info("")
                return

        self.waypoints.append((x, y))
        self.get_logger().info(
            f"Waypoint {len(self.waypoints)} recorded: x={x:.2f}, y={y:.2f}"
        )
        self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")

    def publish_markers(self):
        """Publish markers for waypoints and for Phase 2 circle (if picked)."""
        marker_array = MarkerArray()

        # Phase 2 circle outline (if we have center + radius from map pick)
        if self.circle_display is not None:
            cx, cy, r = self.circle_display
            circle_marker = Marker()
            circle_marker.header.frame_id = f"{self.namespace}/map"
            circle_marker.header.stamp = self.get_clock().now().to_msg()
            circle_marker.ns = "phase2_circle"
            circle_marker.id = 0
            circle_marker.type = Marker.LINE_STRIP
            circle_marker.action = Marker.ADD
            circle_marker.scale.x = 0.05
            circle_marker.color.r = 1.0
            circle_marker.color.g = 0.5
            circle_marker.color.b = 0.0
            circle_marker.color.a = 0.9
            n = 32
            for i in range(n + 1):
                th = 2.0 * math.pi * i / n
                p = Point()
                p.x = cx + r * math.cos(th)
                p.y = cy + r * math.sin(th)
                p.z = 0.05
                circle_marker.points.append(p)
            marker_array.markers.append(circle_marker)

        if not self.waypoints:
            if not marker_array.markers:
                delete_marker = Marker()
                delete_marker.action = Marker.DELETEALL
                marker_array.markers.append(delete_marker)
            self.marker_pub.publish(marker_array)
            return

        for i, (x, y) in enumerate(self.waypoints):
            # Sphere marker for waypoint
            marker = Marker()
            marker.header.frame_id = f"{self.namespace}/map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2  # diameter
            marker.scale.y = 0.2
            marker.scale.z = 0.3  # height
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = f"{self.namespace}/map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 0.5
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3  # text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = str(i + 1)

            marker_array.markers.append(text_marker)

        # Line connecting waypoints
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = f"{self.namespace}/map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "waypoint_path"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # line width
            line_marker.color.r = 0.0
            line_marker.color.g = 0.8
            line_marker.color.b = 1.0
            line_marker.color.a = 0.8

            for x, y in self.waypoints:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                line_marker.points.append(p)

            marker_array.markers.append(line_marker)

        self.marker_pub.publish(marker_array)

    def input_loop(self):
        """Handle keyboard input."""
        while rclpy.ok():
            try:
                user_input = input().strip().lower()

                if user_input == "q":
                    self.get_logger().info("Quitting...")
                    rclpy.shutdown()
                    break

                elif user_input == "p":
                    self.picking_circle = True
                    self.circle_center = None
                    self.get_logger().info(
                        "Pick circle for Phase 2: click the circle CENTER on the map."
                    )

                elif user_input == "c":
                    self.waypoints.clear()
                    self.picking_circle = False
                    self.circle_center = None
                    self.circle_display = None
                    self.publish_markers()  # Clear markers from display
                    self.get_logger().info("Waypoints cleared!")

                elif user_input == "" and len(self.waypoints) > 0:
                    self.get_logger().info(
                        f"Starting navigation with {len(self.waypoints)} waypoints..."
                    )
                    self.navigate()

                elif user_input == "":
                    self.get_logger().info(
                        "No waypoints recorded! Click on the map first."
                    )

            except EOFError:
                break

    def navigate(self):
        """Navigate through recorded waypoints."""
        if not self.waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_waypoints action server not available!")
            return

        # Create poses
        poses = []
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = f"{self.namespace}/map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        # Send goal
        goal = FollowWaypoints.Goal()
        goal.poses = poses

        future = self.waypoint_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted! Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(
            f"Navigating to waypoint {current + 1}/{len(self.waypoints)}"
        )

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints

        if len(missed) == 0:
            self.get_logger().info("All waypoints reached!")
        else:
            self.get_logger().warn(f"Missed waypoints: {list(missed)}")

        self.get_logger().info(
            "Ready for new waypoints. Click on map or press Enter to run again."
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
    node = WaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()