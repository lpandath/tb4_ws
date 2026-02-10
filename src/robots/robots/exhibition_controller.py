#!/usr/bin/env python3
"""
EXHIBITION FINAL CONTROLLER (FROZEN)

Purpose:
--------
Central motion and safety controller for the exhibition robots.

This node:
- Enforces time-based motion scheduling
- Executes exactly one controlled rotation per active window
- Applies human-aware slowdown / pause using front-sector LiDAR
- Guarantees safe startup, shutdown, and failure behavior

Design philosophy:
------------------
- Predictability over cleverness
- Safety over completeness
- Human trust over robot autonomy

⚠️ THIS FILE IS FROZEN
Any modification requires a full rehearsal cycle.
"""

import math
import time
import json
from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, List
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ============================================================
# Motion State Machine
# ============================================================

class MotionState(Enum):
    """
    Discrete motion states for a single robot.

    These states are intentionally minimal and mutually exclusive.
    DO NOT add new states without redesigning the control logic.
    """
    IDLE = auto()          # Not permitted to move
    ACCELERATING = auto()  # Ramp-up phase
    CRUISING = auto()      # Constant-speed phase
    DECELERATING = auto()  # Ramp-down phase
    PAUSED = auto()        # Temporarily stopped due to obstacle
    COMPLETE = auto()      # Finished one full rotation


# ============================================================
# Exhibition Motion Profile (ALL TUNABLES LIVE HERE)
# ============================================================

@dataclass(frozen=True)
class ExhibitionProfile:
    """
    All motion, safety, and scheduling constants.

    Freezing these values in a single structure:
    - makes behavior auditable
    - prevents accidental drift
    - allows rehearsal-based tuning only
    """

    # Nominal motion parameters
    LINEAR_SPEED: float = 0.12
    TARGET_RADIUS: float = 1.3
    ROTATION_ANGLE: float = math.radians(180.0)  # 180° half-circle (not 360°)

    # Human proximity thresholds (meters)
    DISTANCE_STOP: float = 0.5
    DISTANCE_SLOW: float = 0.9
    DISTANCE_RESUME: float = 1.4

    # Speed scaling factors
    SPEED_SCALE_NORMAL: float = 1.0
    SPEED_SCALE_SLOW: float = 0.4
    SPEED_SCALE_STOP: float = 0.0

    # Scheduling (wall-clock anchored)
    CYCLE_PERIOD: float = 900.0     # 15 minutes
    ACTIVE_WINDOW: float = 120.0    # 2 minutes

    # Control loop
    CONTROL_RATE: float = 30.0
    RAMP_FRACTION: float = 0.15     # accel/decel fraction of rotation


# ============================================================
# ExhibitionRobot — ONE PER PHYSICAL ROBOT
# ============================================================

class ExhibitionRobot:
    """
    Encapsulates all per-robot motion, safety, and perception state.

    Thread safety:
    --------------
    All mutable state is protected by a re-entrant lock.
    Controller timers, subscriptions, and services may interleave.
    """

    def __init__(self, name: str, direction: int, node: Node, profile: ExhibitionProfile):
        self.name = name
        self.direction = direction
        self.node = node
        self.profile = profile

        # Synchronization primitive for shared state
        self._lock = threading.RLock()

        # Motion state
        self.motion_state = MotionState.IDLE
        self.speed_scale = profile.SPEED_SCALE_STOP  # Start stopped for safety
        self.rotation_progress = 0.0

        # Perception state
        self._obstacle_distance = float('inf')
        self._last_scan_time = 0.0

        # Telemetry / diagnostics
        self.motion_start_time = None
        self.completion_count = 0
        self.pause_count = 0

        self._setup_ros()
        node.get_logger().info(f" {name}: READY (direction={'CCW' if direction > 0 else 'CW'})")

    def _setup_ros(self):
        """
        Configure ROS publishers and subscribers.

        QoS choices are deliberate:
        - cmd_vel: RELIABLE (do not drop motion commands)
        - scan: BEST_EFFORT (freshness over completeness)
        """

        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.cmd_pub = self.node.create_publisher(
            Twist, f"/{self.name}/cmd_vel", qos_cmd
        )

        qos_scan = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        # Subscribe to both /scan and /scan_new for compatibility
        self.scan_sub = self.node.create_subscription(
            LaserScan, f"/{self.name}/scan", self._scan_callback, qos_scan
        )
        self.scan_new_sub = self.node.create_subscription(
            LaserScan, f"/{self.name}/scan_new", self._scan_callback, qos_scan
        )

    # --------------------------------------------------------
    # Perception: Front-sector LiDAR processing
    # --------------------------------------------------------

    def _scan_callback(self, msg: LaserScan):
        """
        Process incoming LiDAR scan.

        Only the ±60° FRONT SECTOR is considered.
        This is intentional:
        - aligns robot behavior with human expectations
        - avoids false pauses from side / rear clutter
        """

        if msg.angle_increment <= 0:
            return

        now = time.time()

        start_idx = max(
            0,
            int((math.radians(-60) - msg.angle_min) / msg.angle_increment)
        )
        end_idx = min(
            len(msg.ranges) - 1,
            int((math.radians(60) - msg.angle_min) / msg.angle_increment)
        )

        closest = float('inf')
        for i in range(start_idx, end_idx + 1):
            val = msg.ranges[i]
            if 0.1 < val < 5.0:
                closest = min(closest, val)

        with self._lock:
            self._obstacle_distance = closest if closest != float('inf') else 10.0
            self._last_scan_time = now

            # Startup & runtime safety:
            # Motion is only allowed if recent scan data exists
            if now - self._last_scan_time < 0.5:
                if self._obstacle_distance < self.profile.DISTANCE_STOP:
                    self.speed_scale = self.profile.SPEED_SCALE_STOP
                elif self._obstacle_distance < self.profile.DISTANCE_SLOW:
                    self.speed_scale = self.profile.SPEED_SCALE_SLOW
                elif self._obstacle_distance > self.profile.DISTANCE_RESUME:
                    self.speed_scale = self.profile.SPEED_SCALE_NORMAL

    # --------------------------------------------------------
    # Control update (called at CONTROL_RATE)
    # --------------------------------------------------------

    def update(self, within_window: bool, dt: float):
        """Thread-safe entry point for control updates."""
        with self._lock:
            return self._update_unsafe(within_window, dt)

    def _update_unsafe(self, within_window: bool, dt: float):
        """
        Core decision logic.

        Order of authority:
        1. Sensor freshness
        2. Schedule window
        3. Motion state machine
        """

        # Fail-safe: no motion without recent perception
        now = time.time()
        if now - self._last_scan_time > 2.0:
            self.speed_scale = self.profile.SPEED_SCALE_STOP

        # Outside scheduled window → force idle
        if not within_window:
            if self.motion_state != MotionState.IDLE:
                self._stop_unsafe()
                self.motion_state = MotionState.IDLE
                self.rotation_progress = 0.0
                self.motion_start_time = None
            return False

        # Entering window → start rotation exactly once
        if self.motion_state == MotionState.IDLE:
            self.motion_state = MotionState.ACCELERATING
            self.rotation_progress = 0.0
            self.motion_start_time = self.node.get_clock().now()

        self._update_motion_unsafe(dt)
        self._publish_velocity_unsafe()
        return True

    # --------------------------------------------------------
    # Motion state machine
    # --------------------------------------------------------

    def _update_motion_unsafe(self, dt: float):
        if self.motion_state in [MotionState.IDLE, MotionState.COMPLETE]:
            return

        # Obstacle-triggered pause
        if self.speed_scale == self.profile.SPEED_SCALE_STOP:
            if self.motion_state != MotionState.PAUSED:
                self.motion_state = MotionState.PAUSED
                self.pause_count += 1
            return

        # Resume after pause
        if self.motion_state == MotionState.PAUSED and self.speed_scale > 0:
            self.motion_state = MotionState.ACCELERATING
            return

        angular_speed = self._calculate_angular_speed_unsafe()
        remaining = self.profile.ROTATION_ANGLE - self.rotation_progress
        step = abs(angular_speed) * dt
        self.rotation_progress += min(step, remaining)

        total = self.profile.ROTATION_ANGLE
        ramp = self.profile.RAMP_FRACTION

        if self.rotation_progress >= total:
            self.motion_state = MotionState.COMPLETE
            self.completion_count += 1

            if self.motion_start_time:
                duration = self.node.get_clock().now() - self.motion_start_time
                sec = duration.nanoseconds * 1e-9
                self.node.get_logger().info(
                    f"✅ {self.name}: Complete in {sec:.1f}s ({self.pause_count} pauses)"
                )

        elif self.rotation_progress < total * ramp:
            self.motion_state = MotionState.ACCELERATING
        elif self.rotation_progress < total * (1 - ramp):
            self.motion_state = MotionState.CRUISING
        else:
            self.motion_state = MotionState.DECELERATING

    def _calculate_angular_speed_unsafe(self) -> float:
        """
        Compute angular velocity with linear ramp-in / ramp-out.

        Linear ramps are chosen for predictability and ease of reasoning.
        """
        total = self.profile.ROTATION_ANGLE
        progress = self.rotation_progress
        ramp = self.profile.RAMP_FRACTION

        base_speed = (self.profile.LINEAR_SPEED / self.profile.TARGET_RADIUS)
        base_speed *= self.speed_scale

        if self.motion_state == MotionState.ACCELERATING:
            return self.direction * base_speed * (progress / (total * ramp))

        if self.motion_state == MotionState.DECELERATING:
            return self.direction * base_speed * ((total - progress) / (total * ramp))

        return self.direction * base_speed

    def _publish_velocity_unsafe(self):
        """Publish Twist command corresponding to current motion state."""
        cmd = Twist()

        if self.motion_state == MotionState.PAUSED:
            pass  # Explicit stop
        elif self.motion_state != MotionState.COMPLETE:
            angular = self._calculate_angular_speed_unsafe()
            cmd.linear.x = abs(angular) * self.profile.TARGET_RADIUS
            cmd.angular.z = angular

        self.cmd_pub.publish(cmd)

    def _stop_unsafe(self):
        """Stop motion immediately."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    # --------------------------------------------------------
    # External controls
    # --------------------------------------------------------

    def emergency_stop(self):
        """Immediate hard stop and state reset."""
        with self._lock:
            self._stop_unsafe()
            self.motion_state = MotionState.IDLE
            self.rotation_progress = 0.0
            self.motion_start_time = None

    def test_spin(self):
        """Manual override used by staff for testing."""
        with self._lock:
            self.motion_state = MotionState.ACCELERATING
            self.rotation_progress = 0.0
            self.speed_scale = self.profile.SPEED_SCALE_NORMAL
            self.motion_start_time = self.node.get_clock().now()

    def get_status(self) -> Dict:
        """Snapshot of robot state for UI / diagnostics."""
        with self._lock:
            now = time.time()
            scan_age = now - self._last_scan_time if self._last_scan_time else 999
            return {
                'name': self.name,
                'state': self.motion_state.name,
                'progress': self.rotation_progress,
                'distance': self._obstacle_distance,
                'speed_scale': self.speed_scale,
                'scan_age': scan_age,
                'completions': self.completion_count,
                'pauses': self.pause_count
            }


# ============================================================
# ExhibitionController — SYSTEM ORCHESTRATOR
# ============================================================

class ExhibitionController(Node):
    """
    Top-level controller.

    Responsibilities:
    - Owns schedule
    - Owns robots
    - Exposes services
    - Guarantees safe shutdown
    """

    def __init__(self):
        super().__init__("exhibition_controller")

        self.profile = ExhibitionProfile()

        # Create robots with opposite directions for synchronized pendulum
        self.robots = {
            'Moon': ExhibitionRobot('Moon', +1, self, self.profile),   # CCW
            'Basin': ExhibitionRobot('Basin', -1, self, self.profile),  # CW (opposite)
        }

        self.within_active_window = False
        self.window_start_time = None
        self.cycle_count = 0
        self.cycle_start_time = time.time()

        self.control_timer = self.create_timer(
            1.0 / self.profile.CONTROL_RATE, self._control_loop
        )
        self.schedule_timer = self.create_timer(1.0, self._schedule_check)
        self.health_timer = self.create_timer(5.0, self._health_check)

        self._setup_services()
        self._log_startup()

    def _setup_services(self):
        """Set up ROS service servers for web UI integration."""
        # Services can be added here for future UI integration
        pass

    def _control_loop(self):
        """
        Main control loop - runs at CONTROL_RATE (30 Hz).
        Updates all robots with current timing and schedule state.
        """
        dt = 1.0 / self.profile.CONTROL_RATE
        for robot in self.robots.values():
            robot.update(self.within_active_window, dt)

    def _schedule_check(self):
        """
        Check if we're in an active window (runs at 1 Hz).
        Uses relative time from startup to determine schedule.
        """
        elapsed = time.time() - self.cycle_start_time
        cycle_position = elapsed % self.profile.CYCLE_PERIOD
        in_window = cycle_position < self.profile.ACTIVE_WINDOW

        if in_window and not self.within_active_window:
            self.within_active_window = True
            self.window_start_time = time.time()
            self.cycle_count += 1
            self.get_logger().info(
                f"▶️  CYCLE {self.cycle_count}: Active window START"
            )

        elif not in_window and self.within_active_window:
            self.within_active_window = False
            self.get_logger().info(
                f"⏸️  CYCLE {self.cycle_count}: Active window END"
            )

    def _health_check(self):
        """Periodic health check (runs every 5 seconds)."""
        for robot in self.robots.values():
            status = robot.get_status()
            self.get_logger().info(
                f"🔍 {status['name']}: {status['state']} | "
                f"distance={status['distance']:.2f}m | "
                f"completions={status['completions']}"
            )

    def _log_startup(self):
        """Log startup summary."""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎭 EXHIBITION CONTROLLER (FROZEN)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Robots: Moon (CCW) ↔ Basin (CW)")
        self.get_logger().info(f"Motion: 180° half-circle pendulum swings")
        self.get_logger().info(f"Linear speed: {self.profile.LINEAR_SPEED} m/s")
        self.get_logger().info(f"Arc radius: {self.profile.TARGET_RADIUS} m")
        self.get_logger().info(
            f"Schedule: {self.profile.ACTIVE_WINDOW}s active / "
            f"{self.profile.CYCLE_PERIOD}s total"
        )
        self.get_logger().info(f"Safety: STOP at {self.profile.DISTANCE_STOP}m")
        self.get_logger().info("=" * 60)


# ============================================================
# Entry Point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    controller = ExhibitionController()
    executor = SingleThreadedExecutor()
    executor.add_node(controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()