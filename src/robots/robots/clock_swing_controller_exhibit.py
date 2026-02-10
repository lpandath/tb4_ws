#!/usr/bin/env python3
"""
Clock Swing Controller - Pendulum Swinging for Exhibition

Robots perform synchronized half-circle (180°) swings.
Uses three-state proximity logic for battery-efficient operation.
"""

import math
import time
import threading
from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MotionState(Enum):
    IDLE = auto()
    ACCELERATING = auto()
    CRUISING = auto()
    DECELERATING = auto()
    PAUSED = auto()
    COMPLETE = auto()


@dataclass(frozen=True)
class SwingProfile:
    """Parameters for pendulum swing motion."""
    linear_speed: float = 0.35
    target_radius: float = 0.8
    rotation_angle: float = math.pi
    
    distance_stop: float = 0.4
    distance_start: float = 0.75
    speed_scale_normal: float = 1.0
    speed_scale_stop: float = 0.0
    
    control_rate: float = 50.0
    ramp_fraction: float = 0.15
    
    cycle_period: float = 900.0
    active_window: float = 180.0


class SwingRobot:
    """Single robot performing swing motion."""
    
    def __init__(self, name: str, direction: int, node: Node, profile: SwingProfile):
        self.name = name
        self.base_direction = direction
        self.node = node
        self.profile = profile
        
        self._lock = threading.RLock()
        
        self.motion_state = MotionState.IDLE
        self.speed_scale = 0.0
        self.progress = 0.0
        self.swing_direction = direction
        
        self._obstacle_distance = float('inf')
        self._last_scan_time = time.time()
        
        self.completion_count = 0
        self.pause_count = 0
        self.motion_start_time = None
        
        self._setup_ros()
        node.get_logger().info(f"[{name}] swing robot initialized")
    
    def _setup_ros(self):
        """Configure ROS publishers and subscribers."""
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.cmd_pub = self.node.create_publisher(
            Twist, f"/{self.name}/cmd_vel_unstamped", qos_cmd
        )
        
        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.scan_sub = self.node.create_subscription(
            LaserScan, f"/{self.name}/scan", self._scan_callback, qos_scan
        )
    
    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR for proximity-based speed control."""
        if msg.angle_increment <= 0:
            return
        
        start_idx = max(0, int((math.radians(-60) - msg.angle_min) / msg.angle_increment))
        end_idx = min(len(msg.ranges) - 1, int((math.radians(60) - msg.angle_min) / msg.angle_increment))
        
        closest = float('inf')
        for i in range(start_idx, end_idx + 1):
            val = msg.ranges[i]
            if 0.1 < val < 10.0:
                closest = min(closest, val)
        
        with self._lock:
            self._obstacle_distance = closest if closest != float('inf') else 10.0
            self._last_scan_time = time.time()
            
            profile = self.profile
            if self._obstacle_distance > profile.distance_start:
                self.speed_scale = profile.speed_scale_stop
            elif self._obstacle_distance < profile.distance_stop:
                proximity_ratio = self._obstacle_distance / profile.distance_stop
                self.speed_scale = proximity_ratio ** 1.5
            else:
                self.speed_scale = profile.speed_scale_normal
    
    def update(self, within_window: bool, dt: float):
        """Main control update."""
        with self._lock:
            return self._update_unsafe(within_window, dt)
    
    def _update_unsafe(self, within_window: bool, dt: float):
        """Core control logic."""
        now = time.time()
        if now - self._last_scan_time > 2.0:
            self.speed_scale = 0.0
        
        if not within_window:
            if self.motion_state != MotionState.IDLE:
                self._stop_unsafe()
                self.motion_state = MotionState.IDLE
                self.progress = 0.0
            return False
        
        if self.motion_state == MotionState.IDLE:
            self.motion_state = MotionState.ACCELERATING
            self.progress = 0.0
            self.motion_start_time = self.node.get_clock().now()
        
        self._update_motion_unsafe(dt)
        self._publish_velocity_unsafe()
        return True
    
    def _update_motion_unsafe(self, dt: float):
        """Motion state machine."""
        if self.motion_state == MotionState.IDLE:
            return
        
        if self.speed_scale == 0.0:
            if self.motion_state != MotionState.PAUSED:
                self.motion_state = MotionState.PAUSED
                self.pause_count += 1
            return
        
        if self.motion_state == MotionState.PAUSED and self.speed_scale > 0:
            self.motion_state = MotionState.ACCELERATING
            return
        
        profile = self.profile
        angular_speed = (profile.linear_speed / profile.target_radius) * self.speed_scale
        step = abs(angular_speed) * dt
        self.progress = min(self.progress + step, profile.rotation_angle)
        
        total = profile.rotation_angle
        ramp = profile.ramp_fraction * total
        
        if self.progress >= total:
            self.swing_direction *= -1
            self.progress = 0.0
            self.motion_state = MotionState.ACCELERATING
            self.completion_count += 1
            if self.motion_start_time:
                duration = self.node.get_clock().now() - self.motion_start_time
                sec = duration.nanoseconds * 1e-9
                self.node.get_logger().info(
                    f"[{self.name}] swing complete ({sec:.1f}s, {self.pause_count} pauses)"
                )
                self.motion_start_time = self.node.get_clock().now()
        elif self.progress < ramp:
            self.motion_state = MotionState.ACCELERATING
        elif self.progress < total - ramp:
            self.motion_state = MotionState.CRUISING
        else:
            self.motion_state = MotionState.DECELERATING
    
    def _publish_velocity_unsafe(self):
        """Publish Twist command for swing motion."""
        cmd = Twist()
        
        if self.motion_state not in [MotionState.IDLE, MotionState.COMPLETE, MotionState.PAUSED]:
            profile = self.profile
            angular_speed = (profile.linear_speed / profile.target_radius) * self.speed_scale
            cmd.angular.z = self.swing_direction * angular_speed
        
        self.cmd_pub.publish(cmd)
    
    def _stop_unsafe(self):
        """Emergency stop."""
        self.cmd_pub.publish(Twist())
    
    def emergency_stop(self):
        """Public API: hard stop."""
        with self._lock:
            self._stop_unsafe()
            self.motion_state = MotionState.IDLE
            self.progress = 0.0
            self.motion_start_time = None
    
    def get_status(self) -> Dict:
        """Public API: robot status."""
        with self._lock:
            now = time.time()
            return {
                'name': self.name,
                'state': self.motion_state.name,
                'progress': self.progress,
                'distance': self._obstacle_distance,
                'speed_scale': self.speed_scale,
                'completions': self.completion_count,
            }


class ClockSwingControllerExhibit(Node):
    """Clock swing controller for exhibition."""
    
    def __init__(self):
        super().__init__("clock_swing_controller")
        
        self.declare_parameter("robot_names", ["Moon", "Basin"])
        self.declare_parameter("directions", [-1, 1])
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("target_radius", 0.8)
        self.declare_parameter("distance_stop", 0.4)
        self.declare_parameter("distance_start", 0.75)
        self.declare_parameter("cycle_period", 900.0)
        self.declare_parameter("active_window", 180.0)
        self.declare_parameter("control_rate", 50.0)
        
        robot_names = list(self.get_parameter("robot_names").value)
        directions = list(self.get_parameter("directions").value)
        
        profile = SwingProfile(
            linear_speed=float(self.get_parameter("linear_speed").value),
            target_radius=float(self.get_parameter("target_radius").value),
            distance_stop=float(self.get_parameter("distance_stop").value),
            distance_start=float(self.get_parameter("distance_start").value),
            cycle_period=float(self.get_parameter("cycle_period").value),
            active_window=float(self.get_parameter("active_window").value),
            control_rate=float(self.get_parameter("control_rate").value),
        )
        
        self.robots: Dict[str, SwingRobot] = {
            name: SwingRobot(name, direction, self, profile)
            for name, direction in zip(robot_names, directions)
        }
        
        self.profile = profile
        self.within_active_window = False
        self.cycle_start_time = time.time()
        
        self.create_timer(1.0 / profile.control_rate, self._control_loop)
        self.create_timer(1.0, self._schedule_check)
        
        robot_list = ", ".join(robot_names)
        self.get_logger().info(f"Clock swing controller ready for: {robot_list}")
        self._log_config()
    
    def _log_config(self):
        """Log configuration."""
        self.get_logger().info(
            f"Schedule: {self.profile.cycle_period:.0f}s cycles, "
            f"{self.profile.active_window:.0f}s active windows"
        )
        swing_time = self.profile.rotation_angle / (self.profile.linear_speed / self.profile.target_radius)
        self.get_logger().info(
            f"Motion: {self.profile.linear_speed:.2f} m/s, "
            f"radius={self.profile.target_radius:.2f}m, "
            f"~{swing_time:.1f}s per 180° swing"
        )
    
    def _schedule_check(self):
        """Determine if within active window."""
        now = time.time()
        elapsed = now - self.cycle_start_time
        
        cycle_pos = elapsed % self.profile.cycle_period
        self.within_active_window = cycle_pos < self.profile.active_window
        
        if elapsed >= self.profile.cycle_period:
            self.cycle_start_time = now
    
    def _control_loop(self):
        """Main control loop."""
        dt = 1.0 / self.profile.control_rate
        for robot in self.robots.values():
            robot.update(self.within_active_window, dt)
    
    def emergency_stop_all(self):
        """Stop all robots."""
        for robot in self.robots.values():
            robot.emergency_stop()
        self.get_logger().warn("EMERGENCY STOP triggered")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClockSwingControllerExhibit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
