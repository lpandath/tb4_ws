#!/usr/bin/env python3
"""
PHASE 1: CLOCK MOVEMENT CONTROLLER (ANNUAL CLOCK)
Exhibition motion control for Moon and Basin robots.

- In-place semicircle (180°) back-and-forth, like an annual clock: one sweep, then return.
- Visitors walk around the robots to see other sculptures; lidar sees them when in front.
- Audience zone: move only when someone in range (STOP–FAR cm); idle when too close or far.
- Stability: EMA-smoothed distance + hysteresis + min hold times to avoid flip-flop.
"""

import math
import sys
import time
from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MotionState(Enum):
    """Discrete motion states."""

    IDLE = auto()
    ACCELERATING = auto()
    CRUISING = auto()
    DECELERATING = auto()
    PAUSED = auto()
    TURN_AROUND = auto()  # brief full stop between semicircle and return (no jerk)
    COMPLETE = auto()


@dataclass(frozen=True)
class Phase1Profile:
    """Motion and safety parameters."""

    # Rotation: semicircle back-and-forth (annual clock)
    LINEAR_SPEED: float = 0.15  # m/s (unused for in-place)
    TARGET_RADIUS: float = 0.3  # m (unused for in-place)
    ROTATION_ANGLE: float = (
        math.pi
    )  # 180° per sweep; direction flips each sweep → back-and-forth

    # Human proximity (meters). Only perform when audience in range (save battery).
    DISTANCE_STOP: float = 0.20  # too close → stop (safety)
    DISTANCE_FAR: float = (
        0.50  # start moving when distance <= FAR; go idle when > FAR + HYSTERESIS
    )
    FAR_HYSTERESIS: float = (
        0.25  # avoid flip-flop: only go "idle (far)" when distance > FAR + this
    )
    STOP_HYSTERESIS: float = 0.05  # resume moving only when d > STOP + this (e.g. 25+5=30 cm) so boundary is clear

    # Speed scaling (only NORMAL and STOP used; no "slow" band)
    SPEED_SCALE_NORMAL: float = 1.0
    SPEED_SCALE_STOP: float = 0.0

    # Scheduling
    HALF_CIRCLE_DURATION: float = 300.0  # seconds per semicircle (one 180° sweep)
    TOTAL_DURATION: float = 600.0  # total run time (e.g. 10 min)

    # Control
    CONTROL_RATE: float = 30.0
    RAMP_FRACTION: float = 0.15
    # Very gentle (0.05 rad/s) to reduce jump when base goes 0 → moving; if it doesn't move, try 0.08 in launch
    MIN_ANGULAR_SPEED: float = 0.05
    MAX_ANGULAR_SPEED: float = 0.05
    # Smoothing & stability (stop flip-flop from noisy lidar)
    DISTANCE_SMOOTH_ALPHA: float = 0.12  # EMA: lower = smoother, ~8 scans to shift
    AUDIENCE_MIN_HOLD_SEC: float = (
        2.5  # stay "moving" at least this long once in audience zone
    )
    FAR_MIN_HOLD_SEC: float = (
        1.5  # stay "idle (far)" at least this long before allowing move
    )


class Phase1Robot:
    """Per-robot motion and safety controller."""

    def __init__(
        self,
        name: str,
        namespace: str,
        direction: int,
        node: Node,
        profile: Phase1Profile,
        scan_topic: str = "scan_new",
        scan_sector_deg: float = 120.0,
    ):
        self.name = name
        self.namespace = namespace
        self.direction = direction  # +1 or -1 for CW/CCW
        self.node = node
        self.profile = profile
        self.scan_topic = scan_topic
        self.scan_sector_deg = scan_sector_deg

        self._lock = threading.RLock()

        # Motion state (full circles in one direction)
        self.motion_state = MotionState.IDLE
        self.speed_scale = profile.SPEED_SCALE_STOP
        self.rotation_progress = 0.0

        # Perception
        self._obstacle_distance = float("inf")
        self._obstacle_distance_smooth = (
            2.0  # EMA; use for zone logic (starts mid-range)
        )
        self._last_scan_time = 0.0
        self._logged_scan_connected = False
        self._last_was_audience = False
        self._audience_since = 0.0  # time when we last entered "moving" (audience)
        self._far_since = 0.0  # time when we last entered "idle (far)"

        # Telemetry
        self.motion_start_time = None
        self.completion_count = 0
        self.pause_count = 0
        self._turn_around_until = 0.0  # time.time() until we leave TURN_AROUND

        self._setup_ros()
        node.get_logger().info(f" {self.name}: READY")

    def _setup_ros(self):
        """Single topic (cmd_vel_unstamped) to avoid one robot getting double/conflicting commands and jumping."""
        qos_cmd = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
        self.cmd_pub = self.node.create_publisher(
            Twist, f"/{self.namespace}/cmd_vel_unstamped", qos_cmd
        )

        qos_scan = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        scan_topic_full = f"/{self.namespace}/{self.scan_topic}"
        self.scan_sub = self.node.create_subscription(
            LaserScan, scan_topic_full, self._scan_callback, qos_scan
        )

    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR scan - front sector ±scan_sector_deg; closest point in that cone (raw used for stop)."""
        if msg.angle_increment <= 0:
            return

        now = time.time()
        half = self.scan_sector_deg  # ±half degrees from front
        # Extract front sector (wider = see you before you're right at the lidar)
        start_idx = max(
            0, int((math.radians(-half) - msg.angle_min) / msg.angle_increment)
        )
        end_idx = min(
            len(msg.ranges) - 1,
            int((math.radians(half) - msg.angle_min) / msg.angle_increment),
        )

        # Use 0.02 m min so we detect hand very close (was 0.1 and hid 3–10 cm readings)
        closest = float("inf")
        for i in range(start_idx, end_idx + 1):
            val = msg.ranges[i]
            if 0.02 < val < 5.0:
                closest = min(closest, val)

        with self._lock:
            was_first_scan = self._last_scan_time == 0.0
            raw = closest if closest != float("inf") else 10.0
            self._obstacle_distance = raw
            self._last_scan_time = now

            # EMA: only treat 11 cm as outlier when we're far (smooth > 80 cm), so movement can start from 30–80 cm
            alpha = self.profile.DISTANCE_SMOOTH_ALPHA
            smooth_now = self._obstacle_distance_smooth
            stop = self.profile.DISTANCE_STOP
            far = self.profile.DISTANCE_FAR
            far_idle = far + self.profile.FAR_HYSTERESIS
            if raw < 0.15 and smooth_now > far:
                alpha = 0.02  # far and 11 cm spike: don't pull smooth into audience zone
            elif raw < (stop + 0.10) and smooth_now > far_idle:
                alpha = 0.02  # same when well past far threshold
            self._obstacle_distance_smooth = (
                alpha * raw + (1.0 - alpha) * self._obstacle_distance_smooth
            )

            if was_first_scan and not self._logged_scan_connected:
                self._logged_scan_connected = True
                self.node.get_logger().info(
                    f"Scan connected for {self.name} ({self.scan_topic})"
                )

            # Zone logic: clear 25 cm stop, 25+5=30 cm to resume move, 80 cm far
            stop = self.profile.DISTANCE_STOP
            stop_resume = stop + self.profile.STOP_HYSTERESIS  # e.g. 30 cm — must be this far to move again
            far = self.profile.DISTANCE_FAR
            far_idle = far + self.profile.FAR_HYSTERESIS
            hold_audience = self.profile.AUDIENCE_MIN_HOLD_SEC
            hold_far = self.profile.FAR_MIN_HOLD_SEC
            d = self._obstacle_distance_smooth
            raw_d = self._obstacle_distance

            # Too close: stop only when BOTH raw and smooth are below 25 cm (same approach as before, 25 cm instead of 11)
            if raw_d < stop and d < stop:
                self.speed_scale = self.profile.SPEED_SCALE_STOP
                self._last_was_audience = False
                return

            # Audience (move): only when d > 30 cm and d <= 80 cm — so 25–30 cm band keeps previous (no flip)
            in_audience_zone = (d > stop_resume) and (d <= far)
            in_far_zone = d > far_idle

            if in_audience_zone:
                self._last_was_audience = True
                if self._far_since == 0.0 or (now - self._far_since) >= hold_far:
                    was_stopped = self.speed_scale == self.profile.SPEED_SCALE_STOP
                    self.speed_scale = self.profile.SPEED_SCALE_NORMAL
                    if was_stopped:
                        self._audience_since = (
                            now  # transition to moving: start hold timer
                        )
            elif in_far_zone:
                self._last_was_audience = False
                if (
                    now - self._audience_since
                ) >= hold_audience or self._audience_since == 0.0:
                    was_moving = self.speed_scale == self.profile.SPEED_SCALE_NORMAL
                    self.speed_scale = self.profile.SPEED_SCALE_STOP
                    if was_moving:
                        self._far_since = now  # transition to idle: start hold timer
            else:
                # Bands (stop..stop_resume] and (far..far_idle]: keep previous
                if self._last_was_audience:
                    self.speed_scale = self.profile.SPEED_SCALE_NORMAL
                else:
                    self.speed_scale = self.profile.SPEED_SCALE_STOP

    def update(self, rotating: bool, elapsed_time: float, dt: float):
        """Thread-safe control update."""
        with self._lock:
            return self._update_unsafe(rotating, elapsed_time, dt)

    def _update_unsafe(self, rotating: bool, elapsed_time: float, dt: float):
        """Core control logic."""
        now = time.time()

        # No scan yet or stale scan → idle (don't move until we see audience zone)
        if self._last_scan_time == 0.0 or (now - self._last_scan_time > 2.0):
            self.speed_scale = self.profile.SPEED_SCALE_STOP

        # Not rotating → force idle
        if not rotating:
            if self.motion_state != MotionState.IDLE:
                self._stop_unsafe()
                self.motion_state = MotionState.IDLE
                self.rotation_progress = 0.0
                self.motion_start_time = None
            return

        # Only leave IDLE when someone is in audience zone (so we stay idle at start if you're far)
        if self.motion_state == MotionState.IDLE:
            if self.speed_scale > self.profile.SPEED_SCALE_STOP:
                self.motion_state = MotionState.ACCELERATING
                self.rotation_progress = 0.0
                self.motion_start_time = self.node.get_clock().now()
            # else stay IDLE and publish 0 until audience in range

        self._update_motion_unsafe(dt)
        self._publish_velocity_unsafe()

    def _update_motion_unsafe(self, dt: float):
        """Update motion state machine (full circles, repeat until TOTAL_DURATION)."""
        if self.motion_state == MotionState.IDLE:
            return

        # Idle when too close (safety) or far (no audience)
        if self.speed_scale == self.profile.SPEED_SCALE_STOP:
            if self.motion_state != MotionState.PAUSED:
                self.motion_state = MotionState.PAUSED
                self.pause_count += 1
                # Use smooth distance for reason/log so 25 cm boundary is clear (not raw 11 cm)
                d_smooth = self._obstacle_distance_smooth
                reason = (
                    "too close"
                    if d_smooth < self.profile.DISTANCE_STOP
                    else "no audience (far)"
                )
                self.node.get_logger().info(
                    f"{self.name}: idle — {reason} ({d_smooth*100:.0f} cm)"
                )
            return

        # Resume when someone in middle range (audience present)
        if self.motion_state == MotionState.PAUSED and self.speed_scale > 0:
            self.motion_state = MotionState.ACCELERATING
            self.node.get_logger().info(
                f"{self.name}: audience in range ({self._obstacle_distance_smooth*100:.0f} cm), moving"
            )
            return

        # End of semicircle: full stop for 1 s before return sweep (avoids jerk / drifting)
        if self.motion_state == MotionState.TURN_AROUND:
            if time.time() >= self._turn_around_until:
                self.completion_count += 1
                self.rotation_progress = 0.0
                self.motion_state = MotionState.ACCELERATING
                self.node.get_logger().info(
                    f"{self.name}: starting return sweep ({self.completion_count})"
                )
            return

        # Update rotation progress
        angular_speed = self._calculate_angular_speed_unsafe()
        total = self.profile.ROTATION_ANGLE
        remaining = total - self.rotation_progress
        step = abs(angular_speed) * dt
        self.rotation_progress += min(step, remaining)

        ramp = self.profile.RAMP_FRACTION

        # Semicircle done: full stop (TURN_AROUND) then return sweep next
        if self.rotation_progress >= total:
            self.rotation_progress = total  # cap
            self.motion_state = MotionState.TURN_AROUND
            self._turn_around_until = time.time() + 1.0
            return

        if self.rotation_progress < total * ramp:
            self.motion_state = MotionState.ACCELERATING
        elif self.rotation_progress < total * (1 - ramp):
            self.motion_state = MotionState.CRUISING
        else:
            self.motion_state = MotionState.DECELERATING

    def _calculate_angular_speed_unsafe(self) -> float:
        """Angular velocity (rad/s). Ramp shape for smoothness; clamp to min_ang so base actually moves (many ignore tiny commands)."""
        total = self.profile.ROTATION_ANGLE
        progress = self.rotation_progress
        ramp = self.profile.RAMP_FRACTION

        base_speed = self.profile.ROTATION_ANGLE / self.profile.HALF_CIRCLE_DURATION
        base_speed *= self.speed_scale

        direction = (
            self.direction if (self.completion_count % 2 == 0) else -self.direction
        )
        min_ang = self.profile.MIN_ANGULAR_SPEED
        max_ang = self.profile.MAX_ANGULAR_SPEED

        if self.motion_state == MotionState.ACCELERATING:
            ramp_dist = total * ramp
            factor = (progress / ramp_dist) if ramp_dist > 0 else 0.0
            factor = max(factor, 0.02)
            raw = direction * base_speed * factor
        elif self.motion_state == MotionState.DECELERATING:
            raw = direction * base_speed * ((total - progress) / (total * ramp))
        else:
            raw = direction * max(base_speed, min_ang)
        # Clamp to [min_ang, max_ang] so base responds but never gets a big command (gentle in-place only)
        if raw > 0:
            return max(min_ang, min(raw, max_ang))
        if raw < 0:
            return min(-min_ang, max(raw, -max_ang))
        return 0.0

    def _publish_velocity_unsafe(self):
        """Publish Twist: in-place only (linear=0, angular.z). If robot still jumps: stop any other cmd_vel publisher (Nav2/teleop); try max_angular_speed:=0.03 in launch."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        if self.motion_state not in [
            MotionState.PAUSED,
            MotionState.TURN_AROUND,
            MotionState.COMPLETE,
            MotionState.IDLE,
        ]:
            cmd.angular.z = self._calculate_angular_speed_unsafe()
        self.cmd_pub.publish(cmd)

    def _stop_unsafe(self):
        """Send explicit zero velocity (no drift)."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def emergency_stop(self):
        """Immediate stop."""
        with self._lock:
            self._stop_unsafe()
            self.motion_state = MotionState.IDLE
            self.rotation_progress = 0.0

    def get_status(self) -> Dict:
        """Return robot status."""
        with self._lock:
            now = time.time()
            scan_age = now - self._last_scan_time if self._last_scan_time else 999
            return {
                "name": self.name,
                "state": self.motion_state.name,
                "progress": self.rotation_progress,
                "distance": self._obstacle_distance,
                "speed_scale": self.speed_scale,
                "scan_age": scan_age,
                "completions": self.completion_count,
                "pauses": self.pause_count,
            }


class Phase1Controller(Node):
    """System orchestrator for Phase 1 clock movement."""

    def __init__(self):
        super().__init__("phase1_controller")

        # Allow runtime overrides from launch parameters (keeps dataclass defaults)
        default_profile = Phase1Profile()
        # Declare parameters with defaults from the dataclass
        self.declare_parameter("linear_speed", default_profile.LINEAR_SPEED)
        self.declare_parameter("target_radius", default_profile.TARGET_RADIUS)
        self.declare_parameter("rotation_angle", default_profile.ROTATION_ANGLE)
        self.declare_parameter("distance_stop", default_profile.DISTANCE_STOP)
        self.declare_parameter("distance_far", default_profile.DISTANCE_FAR)
        self.declare_parameter("far_hysteresis", default_profile.FAR_HYSTERESIS)
        self.declare_parameter("stop_hysteresis", default_profile.STOP_HYSTERESIS)
        self.declare_parameter("speed_scale_normal", default_profile.SPEED_SCALE_NORMAL)
        self.declare_parameter("speed_scale_stop", default_profile.SPEED_SCALE_STOP)
        self.declare_parameter(
            "half_circle_duration", default_profile.HALF_CIRCLE_DURATION
        )
        self.declare_parameter("total_duration", default_profile.TOTAL_DURATION)
        self.declare_parameter("control_rate", default_profile.CONTROL_RATE)
        self.declare_parameter("ramp_fraction", default_profile.RAMP_FRACTION)
        self.declare_parameter("min_angular_speed", default_profile.MIN_ANGULAR_SPEED)
        self.declare_parameter("max_angular_speed", default_profile.MAX_ANGULAR_SPEED)
        self.declare_parameter(
            "distance_smooth_alpha", default_profile.DISTANCE_SMOOTH_ALPHA
        )
        self.declare_parameter(
            "audience_min_hold_sec", default_profile.AUDIENCE_MIN_HOLD_SEC
        )
        self.declare_parameter("far_min_hold_sec", default_profile.FAR_MIN_HOLD_SEC)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("scan_topic", "scan_new")
        self.declare_parameter("scan_sector_deg", 120.0)
        self.declare_parameter("robots", "Moon")

        # Read parameters and construct profile
        self.profile = Phase1Profile(
            LINEAR_SPEED=self.get_parameter("linear_speed").value,
            TARGET_RADIUS=self.get_parameter("target_radius").value,
            ROTATION_ANGLE=self.get_parameter("rotation_angle").value,
            DISTANCE_STOP=self.get_parameter("distance_stop").value,
            DISTANCE_FAR=self.get_parameter("distance_far").value,
            FAR_HYSTERESIS=self.get_parameter("far_hysteresis").value,
            STOP_HYSTERESIS=self.get_parameter("stop_hysteresis").value,
            SPEED_SCALE_NORMAL=self.get_parameter("speed_scale_normal").value,
            SPEED_SCALE_STOP=self.get_parameter("speed_scale_stop").value,
            HALF_CIRCLE_DURATION=self.get_parameter("half_circle_duration").value,
            TOTAL_DURATION=self.get_parameter("total_duration").value,
            CONTROL_RATE=self.get_parameter("control_rate").value,
            RAMP_FRACTION=self.get_parameter("ramp_fraction").value,
            MIN_ANGULAR_SPEED=self.get_parameter("min_angular_speed").value,
            MAX_ANGULAR_SPEED=self.get_parameter("max_angular_speed").value,
            DISTANCE_SMOOTH_ALPHA=self.get_parameter("distance_smooth_alpha").value,
            AUDIENCE_MIN_HOLD_SEC=self.get_parameter("audience_min_hold_sec").value,
            FAR_MIN_HOLD_SEC=self.get_parameter("far_min_hold_sec").value,
        )

        scan_topic = self.get_parameter("scan_topic").value
        scan_sector_deg = self.get_parameter("scan_sector_deg").value
        robot_names = self.get_parameter("robots").value
        if isinstance(robot_names, str):
            robot_names = [s.strip() for s in robot_names.split(",") if s.strip()] or [
                "Moon"
            ]
        elif not isinstance(robot_names, list):
            robot_names = [robot_names] if robot_names else ["Moon"]
        # Name -> (namespace, direction): Moon CW (+1), Basin CCW (-1); same speed for both
        robot_config = {"Moon": ("Moon", 1), "Basin": ("Basin", -1)}
        self.robots = {}
        for name in robot_names:
            if name not in robot_config:
                self.get_logger().warn(
                    f"Unknown robot '{name}', skipping. Known: Moon, Basin."
                )
                continue
            ns, direction = robot_config[name]
            self.robots[name] = Phase1Robot(
                name,
                ns,
                direction,
                self,
                self.profile,
                scan_topic,
                scan_sector_deg,
            )
        if not self.robots:
            raise ValueError(
                "No valid robots configured. Use robots:=['Moon'] or robots:=['Moon','Basin']"
            )

        self.rotating = False
        self.start_time = None

        self.control_timer = self.create_timer(
            1.0 / self.profile.CONTROL_RATE, self._control_loop
        )

        auto_start = self.get_parameter("auto_start").value
        if auto_start:
            self.get_logger().info("Auto-start: rotation will begin in 3s...")
            self.create_timer(3.0, self._auto_start_once)

        self.get_logger().info("=" * 60)
        self.get_logger().info("PHASE 1: CLOCK MOVEMENT CONTROLLER")
        self.get_logger().info("=" * 60)
        if "Moon" in self.robots:
            self.get_logger().info("🌙 Moon: Clockwise")
        if "Basin" in self.robots:
            self.get_logger().info("🪨 Basin: Counter-clockwise")
        self.get_logger().info(
            f"Duration: {self.profile.TOTAL_DURATION}s | "
            f"Full circle every {self.profile.HALF_CIRCLE_DURATION}s (clock: alternate direction each lap)"
        )
        scan_topic = self.get_parameter("scan_topic").value
        sector = self.get_parameter("scan_sector_deg").value
        far_idle = self.profile.DISTANCE_FAR + self.profile.FAR_HYSTERESIS
        self.get_logger().info(
            f"Audience: move {self.profile.DISTANCE_STOP*100:.0f}–{self.profile.DISTANCE_FAR*100:.0f}cm, "
            f"idle when <{self.profile.DISTANCE_STOP*100:.0f}cm or >{far_idle*100:.0f}cm (hysteresis)"
        )
        self.get_logger().info(
            f"Rotation speed: {self.profile.MAX_ANGULAR_SPEED:.3f} rad/s (semicircle in ~{math.pi / self.profile.MAX_ANGULAR_SPEED:.0f}s)"
        )
        self.get_logger().info("")
        self.get_logger().info("Commands:")
        self.get_logger().info(
            "  [Enter] - START rotation (only when run with 'ros2 run' in a terminal)"
        )
        self.get_logger().info("  [s]     - STOP  [q] - QUIT")
        self.get_logger().info(
            '  Or call service: ros2 service call /phase1_controller/start std_srvs/srv/Empty "{}"'
        )
        self.get_logger().info("=" * 60)

        self._fallback_start_timer = None
        if not sys.stdin.isatty() and not auto_start:
            self.get_logger().info(
                "No terminal input - rotation will auto-start in 5s (or call start service now)."
            )
            self._fallback_start_timer = self.create_timer(5.0, self._auto_start_once)

        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self.input_thread.start()

    def _auto_start_once(self):
        """One-shot: start rotation (used when auto_start is true or no-TTY fallback)."""
        if not self.rotating:
            self.get_logger().info("  Auto-starting Phase 1...")
            self.rotating = True
            self.start_time = time.time()
        if getattr(self, "_fallback_start_timer", None) is not None:
            self._fallback_start_timer.cancel()
            self._fallback_start_timer = None

    def _control_loop(self):
        """Main control loop."""
        if not self.rotating or self.start_time is None:
            return

        elapsed = time.time() - self.start_time

        # Check if total duration exceeded
        if elapsed > self.profile.TOTAL_DURATION:
            self.get_logger().info("Phase 1 complete!")
            self.rotating = False
            for robot in self.robots.values():
                robot.emergency_stop()
            return

        # Update all robots
        dt = 1.0 / self.profile.CONTROL_RATE
        for robot in self.robots.values():
            robot.update(self.rotating, elapsed, dt)

    def _input_loop(self):
        """Handle user commands."""
        while rclpy.ok():
            try:
                cmd = input().strip().lower()

                if cmd == "":
                    if self.rotating:
                        self.get_logger().info("Already rotating...")
                    else:
                        self.get_logger().info("  Starting Phase 1...")
                        self.rotating = True
                        self.start_time = time.time()

                elif cmd == "s":
                    self.get_logger().info("Stopping rotation...")
                    self.rotating = False
                    for robot in self.robots.values():
                        robot.emergency_stop()

                elif cmd == "status":
                    for robot in self.robots.values():
                        status = robot.get_status()
                        self.get_logger().info(
                            f"{status['name']}: {status['state']} | "
                            f"Distance: {status['distance']:.2f}m | "
                            f"Pauses: {status['pauses']}"
                        )

                elif cmd == "q":
                    self.get_logger().info("Quitting...")
                    rclpy.shutdown()
                    break

            except EOFError:
                break


def main(args=None):
    rclpy.init(args=args)
    controller = Phase1Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()