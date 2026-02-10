#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


@dataclass
class RobotConfig:
    name: str
    direction: int
    scan_topic: str
    cmd_vel_topic: str


@dataclass
class RobotRuntime:
    distance: float = float("inf")
    state: str = "IDLE_FAR"
    progress: float = 0.0
    done: bool = False


class ClockSwingController(Node):
    def __init__(self) -> None:
        super().__init__("clock_swing_controller")

        self.declare_parameter("robot_names", ["Moon", "Basin"])
        self.declare_parameter("directions", [-1, 1])
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("scan_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("cmd_vel_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("radius", 0.8)
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("stop_distance", 0.8)
        self.declare_parameter("start_distance", 1.2)
        self.declare_parameter("scan_angle_min_deg", -90.0)
        self.declare_parameter("scan_angle_max_deg", 90.0)
        self.declare_parameter("scan_min_range", 0.05)
        self.declare_parameter("scan_max_range", 5.0)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("require_both_active_to_start", True)
        self.declare_parameter("auto_restart", False)
        self.declare_parameter("debug_log", False)

        robot_names = list(self.get_parameter("robot_names").value)
        directions = list(self.get_parameter("directions").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        scan_topics = list(self.get_parameter("scan_topics").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        cmd_vel_topics = list(self.get_parameter("cmd_vel_topics").value)

        if len(directions) != len(robot_names):
            raise ValueError("directions must match robot_names length")

        self.radius = float(self.get_parameter("radius").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.start_distance = float(self.get_parameter("start_distance").value)
        self.scan_angle_min_deg = float(self.get_parameter("scan_angle_min_deg").value)
        self.scan_angle_max_deg = float(self.get_parameter("scan_angle_max_deg").value)
        self.scan_min_range = float(self.get_parameter("scan_min_range").value)
        self.scan_max_range = float(self.get_parameter("scan_max_range").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.require_both_active_to_start = bool(
            self.get_parameter("require_both_active_to_start").value
        )
        self.auto_restart = bool(self.get_parameter("auto_restart").value)
        self.debug_log = bool(self.get_parameter("debug_log").value)

        self.angular_speed = 0.0 if self.radius <= 0.0 else self.linear_speed / self.radius
        self.target_angle = math.pi

        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.robots: Dict[str, RobotConfig] = {}
        self.runtime: Dict[str, RobotRuntime] = {}
        self.scan_subs: Dict[str, object] = {}
        self.cmd_pubs: Dict[str, object] = {}

        for index, (name, direction) in enumerate(zip(robot_names, directions)):
            scan_topic_name = scan_topic
            cmd_topic_name = cmd_vel_topic
            if len(scan_topics) == len(robot_names):
                scan_topic_name = str(scan_topics[index])
            if len(cmd_vel_topics) == len(robot_names):
                cmd_topic_name = str(cmd_vel_topics[index])
            config = RobotConfig(
                name=name,
                direction=int(direction),
                scan_topic=f"/{name}/{scan_topic_name}",
                cmd_vel_topic=f"/{name}/{cmd_topic_name}",
            )
            self.robots[name] = config
            self.runtime[name] = RobotRuntime()

            self.scan_subs[name] = self.create_subscription(
                LaserScan,
                config.scan_topic,
                lambda msg, robot_name=name: self._scan_callback(robot_name, msg),
                qos_scan,
            )
            self.cmd_pubs[name] = self.create_publisher(
                Twist,
                config.cmd_vel_topic,
                qos_cmd,
            )

        self.group_started = False
        self.last_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.control_rate, self._control_loop)

        robot_list = ", ".join(robot_names)
        self.get_logger().info(f"Clock swing controller ready for: {robot_list}")

    def _scan_callback(self, robot_name: str, msg: LaserScan) -> None:
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        min_deg = self.scan_angle_min_deg
        max_deg = self.scan_angle_max_deg
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)

        if angle_increment <= 0.0:
            return

        start_index = max(0, int((min_rad - angle_min) / angle_increment))
        end_index = min(len(msg.ranges) - 1, int((max_rad - angle_min) / angle_increment))

        if end_index < start_index:
            start_index, end_index = end_index, start_index

        closest = float("inf")
        for idx in range(start_index, end_index + 1):
            dist = msg.ranges[idx]
            if math.isfinite(dist) and self.scan_min_range <= dist <= self.scan_max_range:
                if dist < closest:
                    closest = dist

        self.runtime[robot_name].distance = closest

    def _evaluate_state(self, runtime: RobotRuntime) -> str:
        if runtime.done:
            return "DONE"
        if runtime.distance < self.stop_distance:
            return "PAUSED_NEAR"
        if runtime.distance <= self.start_distance:
            return "ACTIVE"
        return "IDLE_FAR"

    def _should_start_group(self) -> bool:
        if self.group_started:
            return True
        active_count = sum(1 for runtime in self.runtime.values() if runtime.state == "ACTIVE")
        if self.require_both_active_to_start:
            return active_count == len(self.runtime)
        return active_count > 0

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        for runtime in self.runtime.values():
            runtime.state = self._evaluate_state(runtime)

        if not self.group_started:
            self.group_started = self._should_start_group()

        for name, config in self.robots.items():
            runtime = self.runtime[name]
            cmd = Twist()

            if runtime.done:
                self.cmd_pubs[name].publish(cmd)
                continue

            if not self.group_started:
                self.cmd_pubs[name].publish(cmd)
                continue

            if runtime.state != "ACTIVE":
                self.cmd_pubs[name].publish(cmd)
                continue

            runtime.progress += abs(self.angular_speed) * dt
            if runtime.progress >= self.target_angle:
                runtime.done = True
                self.cmd_pubs[name].publish(Twist())
                continue

            cmd.linear.x = self.linear_speed
            cmd.angular.z = float(config.direction) * self.angular_speed
            self.cmd_pubs[name].publish(cmd)

        if self.debug_log:
            log_dt = (now - self.last_log_time).nanoseconds * 1e-9
            if log_dt >= 1.0:
                self.last_log_time = now
                for name, runtime in self.runtime.items():
                    self.get_logger().info(
                        f"{name}: dist={runtime.distance:.2f} state={runtime.state} "
                        f"progress={runtime.progress:.2f} done={runtime.done}"
                    )

        if all(runtime.done for runtime in self.runtime.values()):
            if self.auto_restart:
                for runtime in self.runtime.values():
                    runtime.done = False
                    runtime.progress = 0.0
                self.group_started = False
            else:
                self.group_started = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockSwingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
