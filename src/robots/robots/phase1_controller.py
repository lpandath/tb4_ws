#!/usr/bin/env python3
"""
PHASE 1: CLOCK MOVEMENT CONTROLLER (ANNUAL CLOCK)
=================================================
- Robot always swings ~160 degrees back and forth (like a pendulum)
- Stops ONLY when someone is within 80cm (360° safety detection)
- Resumes when person moves away beyond 90cm
"""

import math
import signal
import time
import threading
from enum import Enum, auto
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty as EmptySrv

SCAN_TIMEOUT = 10.0      # Stop robot if no scan data for this many seconds
SCAN_RESUBSCRIBE = 15.0  # Re-create subscription to force DDS re-discovery


class MotionState(Enum):
    IDLE = auto()
    MOVING = auto()
    PAUSED = auto()       # Stopped because person too close
    TURN_AROUND = auto()  # Pause between sweeps


@dataclass(frozen=True)
class MotionProfile:
    ROTATION_ANGLE: float = 2.79       # ~160 degrees
    HALF_SWEEP_DURATION: float = 300.0 # Seconds per sweep (5 min)
    TOTAL_DURATION: float = 600.0      # Total run time (10 min)
    MAX_ANGULAR_SPEED: float = 0.06    # rad/s
    MIN_ANGULAR_SPEED: float = 0.01    # rad/s (smooth start)
    MAX_ACCEL_ANGULAR: float = 0.04    # Smooth acceleration
    CONTROL_RATE: float = 25.0         # Hz


@dataclass(frozen=True)
class SafetyProfile:
    CLOSE_STOP: float = 0.80       # < 80cm from any direction: stop
    CLOSE_RESUME: float = 0.90     # > 90cm: resume (hysteresis)
    LIDAR_MIN_RANGE: float = 0.15  # Ignore < 15cm (noise)
    LIDAR_MAX_RANGE: float = 5.0   # Ignore > 5m
    EMERGENCY_DIST: float = 0.20   # < 20cm: hard emergency stop


def load_parameters(node: Node):
    node.declare_parameter("close_stop", 0.80)
    node.declare_parameter("close_resume", 0.90)
    node.declare_parameter("rotation_angle", 2.79)
    node.declare_parameter("half_sweep_duration", 300.0)
    node.declare_parameter("total_duration", 600.0)
    node.declare_parameter("max_angular_speed", 0.06)
    node.declare_parameter("min_angular_speed", 0.03)
    node.declare_parameter("max_accel_angular", 0.04)
    node.declare_parameter("control_rate", 25.0)
    node.declare_parameter("lidar_min_range", 0.15)
    node.declare_parameter("lidar_max_range", 5.0)
    node.declare_parameter("scan_topic", "scan")
    node.declare_parameter("auto_start", False)
    node.declare_parameter("duty_cycle", False)
    node.declare_parameter("active_duration", 120.0)    # 2 min moving
    node.declare_parameter("rest_duration", 480.0)     # 8 min rest

    motion = MotionProfile(
        ROTATION_ANGLE=node.get_parameter("rotation_angle").value,
        HALF_SWEEP_DURATION=node.get_parameter("half_sweep_duration").value,
        TOTAL_DURATION=node.get_parameter("total_duration").value,
        MAX_ANGULAR_SPEED=node.get_parameter("max_angular_speed").value,
        MIN_ANGULAR_SPEED=node.get_parameter("min_angular_speed").value,
        MAX_ACCEL_ANGULAR=node.get_parameter("max_accel_angular").value,
        CONTROL_RATE=node.get_parameter("control_rate").value,
    )

    safety = SafetyProfile(
        CLOSE_STOP=node.get_parameter("close_stop").value,
        CLOSE_RESUME=node.get_parameter("close_resume").value,
        LIDAR_MIN_RANGE=node.get_parameter("lidar_min_range").value,
        LIDAR_MAX_RANGE=node.get_parameter("lidar_max_range").value,
    )

    return motion, safety


class Phase1Robot:
    """
    Controls ONE robot. Always swings back and forth.
    Only stops when 360° lidar detects someone within 50cm.
    """

    def __init__(self, name, namespace, direction, node, motion, safety, scan_topic):
        self.name = name
        self.namespace = namespace
        self.direction = direction
        self.node = node
        self.motion = motion
        self.safety = safety
        self.scan_topic = scan_topic

        self._lock = threading.RLock()
        self._created_time = time.time()

        self.state = MotionState.IDLE
        self.rotation_progress = 0.0
        self.completion_count = 0

        # Safety: closest object in 360°
        self._closest_360 = float("inf")
        self._is_close = False  # Is someone within close_stop distance?
        self._last_scan_time = 0.0
        self._scan_count = 0
        self._scan_lost = False  # True when scan data has timed out
        self._scan_lost_time = 0.0  # When scan was first lost
        self._last_resubscribe = 0.0
        self._scan_subs = []  # Track subscriptions for re-creation
        self._last_status_time = 0.0

        # Velocity
        self._last_omega = 0.0
        self._max_domega = motion.MAX_ACCEL_ANGULAR / motion.CONTROL_RATE

        # Timers
        self._turn_around_until = 0.0
        self._emergency_until = 0.0
        self._resume_until = 0.0  # Soft-start delay after resuming from pause

        self._setup_ros()

    def _setup_ros(self):
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
        )
        # Publish Twist to cmd_vel_unstamped (for create3_repub if running)
        self.cmd_pub = self.node.create_publisher(
            Twist, f"/{self.namespace}/cmd_vel_unstamped", qos_cmd
        )
        # Also publish TwistStamped to cmd_vel (direct to Create 3 base)
        self.cmd_stamped_pub = self.node.create_publisher(
            TwistStamped, f"/{self.namespace}/cmd_vel", qos_cmd
        )

        self._subscribe_scan()

        # Lidar motor service clients
        self._start_motor_cli = self.node.create_client(
            EmptySrv, f"/{self.namespace}/start_motor"
        )
        self._stop_motor_cli = self.node.create_client(
            EmptySrv, f"/{self.namespace}/stop_motor"
        )
        self._lidar_on = True

    def start_lidar(self):
        """Start lidar motor and re-subscribe to scan data."""
        if not self._lidar_on:
            self._lidar_on = True
            self._scan_count = 0
            self._last_scan_time = 0.0
            self._scan_lost = False
            if self._start_motor_cli.service_is_ready():
                self._start_motor_cli.call_async(EmptySrv.Request())
                self.node.get_logger().info(f"{self.name}: lidar motor START")
            else:
                self.node.get_logger().warn(f"{self.name}: start_motor service not ready")
            self._subscribe_scan()

    def stop_lidar(self):
        """Stop lidar motor to save battery."""
        if self._lidar_on:
            self._lidar_on = False
            if self._stop_motor_cli.service_is_ready():
                self._stop_motor_cli.call_async(EmptySrv.Request())
                self.node.get_logger().info(f"{self.name}: lidar motor STOP (saving battery)")
            else:
                self.node.get_logger().warn(f"{self.name}: stop_motor service not ready")

    def _subscribe_scan(self):
        """Create scan subscriptions. Can be called again to force DDS re-discovery."""
        # Destroy old subscriptions if any
        for sub in self._scan_subs:
            self.node.destroy_subscription(sub)
        self._scan_subs.clear()

        for reliability in [QoSReliabilityPolicy.BEST_EFFORT, QoSReliabilityPolicy.RELIABLE]:
            qos = QoSProfile(reliability=reliability,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=5)
            sub = self.node.create_subscription(
                LaserScan, f"/{self.namespace}/{self.scan_topic}",
                self._scan_callback, qos,
            )
            self._scan_subs.append(sub)

    def _scan_callback(self, msg: LaserScan):
        """360° scan: find closest object in any direction for safety."""
        if msg.angle_increment <= 0:
            return

        now = time.time()
        closest = float("inf")
        for r in msg.ranges:
            if self.safety.LIDAR_MIN_RANGE < r < self.safety.LIDAR_MAX_RANGE:
                if r < closest:
                    closest = r

        with self._lock:
            self._closest_360 = closest if closest != float("inf") else self.safety.LIDAR_MAX_RANGE
            self._last_scan_time = now
            self._scan_count += 1

            # Recovered from scan loss
            if self._scan_lost:
                self._scan_lost = False
                self.node.get_logger().warn(f"{self.name}: scan data RECOVERED")

            if self._scan_count == 1:
                self.node.get_logger().info(
                    f"{self.name}: first scan! closest={self._closest_360:.2f}m"
                )

            # Emergency stop at 20cm
            if self._closest_360 < self.safety.EMERGENCY_DIST:
                self._is_close = True
                self._stop()
                self._emergency_until = now + 0.3
                return

            # Hysteresis: stop at 80cm, resume at 90cm
            if self._is_close:
                if self._closest_360 >= self.safety.CLOSE_RESUME:
                    self._is_close = False
            else:
                if self._closest_360 < self.safety.CLOSE_STOP:
                    self._is_close = True


    def update(self, rotating, dt):
        with self._lock:
            now = time.time()

            # Scan watchdog: stop if no scan data for SCAN_TIMEOUT seconds
            scan_age = (now - self._last_scan_time) if self._last_scan_time > 0 else (now - self._start_time if hasattr(self, '_start_time') else 999)
            if self._last_scan_time == 0 and self._scan_count == 0:
                # Never received first scan — use time since setup
                scan_age = now - getattr(self, '_created_time', now)
            if scan_age > SCAN_TIMEOUT:
                if not self._scan_lost:
                    self._scan_lost = True
                    self._scan_lost_time = now
                    self.node.get_logger().error(
                        f"{self.name}: NO SCAN DATA for {SCAN_TIMEOUT}s! Stopping for safety."
                    )
                # Auto-recovery: re-subscribe every SCAN_RESUBSCRIBE seconds
                if now - self._scan_lost_time >= SCAN_RESUBSCRIBE and now - self._last_resubscribe >= SCAN_RESUBSCRIBE:
                    self._last_resubscribe = now
                    self.node.get_logger().warn(
                        f"{self.name}: re-creating scan subscription (DDS recovery)..."
                    )
                    self._subscribe_scan()
                self._stop()
                self.state = MotionState.PAUSED
                return

            # Periodic status log (every 30s)
            if now - self._last_status_time >= 30.0:
                self._last_status_time = now
                direction = "CW" if (self.direction if self.completion_count % 2 == 0 else -self.direction) > 0 else "CCW"
                self.node.get_logger().info(
                    f"{self.name}: state={self.state.name} sweep={self.completion_count} "
                    f"dir={direction} closest={self._closest_360:.2f}m "
                    f"scans={self._scan_count}"
                )

            # Emergency cooldown
            if now < self._emergency_until:
                self.state = MotionState.PAUSED
                return

            # Master switch
            if not rotating:
                self._stop()
                self.state = MotionState.IDLE
                self.rotation_progress = 0.0
                return

            # Person too close → pause
            if self._is_close:
                self._stop()
                self.state = MotionState.PAUSED
                self._resume_until = 0.0  # Reset so next resume gets fresh delay
                return

            # Resume from pause — soft start with 1s delay
            if self.state in (MotionState.IDLE, MotionState.PAUSED):
                if self._resume_until == 0.0:
                    self._resume_until = now + 1.0
                if now < self._resume_until:
                    return  # Wait before moving
                self._resume_until = 0.0
                self.state = MotionState.MOVING

            # Turn around pause between sweeps
            if self.state == MotionState.TURN_AROUND:
                if now >= self._turn_around_until:
                    self.completion_count += 1
                    self.rotation_progress = 0.0
                    self.state = MotionState.MOVING
                    self.node.get_logger().info(
                        f"{self.name}: sweep {self.completion_count} starting"
                    )
                return

            # Normal motion
            omega = self._calc_omega()
            step = abs(omega) * dt
            remaining = self.motion.ROTATION_ANGLE - self.rotation_progress
            self.rotation_progress += min(step, remaining)

            if self.rotation_progress >= self.motion.ROTATION_ANGLE:
                self.rotation_progress = self.motion.ROTATION_ANGLE
                self.state = MotionState.TURN_AROUND
                self._turn_around_until = now + 1.0
                self._stop()
                self.node.get_logger().info(
                    f"{self.name}: sweep done, reversing in 1s"
                )
                return

            self._publish(omega)

    def _calc_omega(self):
        base = self.motion.ROTATION_ANGLE / self.motion.HALF_SWEEP_DURATION
        direction = self.direction if self.completion_count % 2 == 0 else -self.direction
        raw = direction * base

        if raw > 0:
            return max(self.motion.MIN_ANGULAR_SPEED,
                       min(raw, self.motion.MAX_ANGULAR_SPEED))
        else:
            return min(-self.motion.MIN_ANGULAR_SPEED,
                       max(raw, -self.motion.MAX_ANGULAR_SPEED))

    def _publish(self, target):
        delta = max(-self._max_domega, min(self._max_domega, target - self._last_omega))
        omega = self._last_omega + delta
        self._last_omega = omega
        msg = Twist()
        msg.angular.z = omega
        self.cmd_pub.publish(msg)
        stamped = TwistStamped()
        stamped.twist.angular.z = omega
        self.cmd_stamped_pub.publish(stamped)

    def _stop(self):
        self._last_omega = 0.0
        self.cmd_pub.publish(Twist())
        self.cmd_stamped_pub.publish(TwistStamped())

    def emergency_stop(self):
        self._stop()
        self.state = MotionState.IDLE
        self.rotation_progress = 0.0


class Phase1Controller(Node):

    def __init__(self):
        super().__init__("phase1_controller")

        self.motion, self.safety = load_parameters(self)

        self.get_logger().info("=== Phase 1 Configuration ===")
        self.get_logger().info(f"  Safety stop: < {self.safety.CLOSE_STOP}m (360°)")
        self.get_logger().info(f"  Safety resume: > {self.safety.CLOSE_RESUME}m")
        self.get_logger().info(f"  Rotation: {math.degrees(self.motion.ROTATION_ANGLE):.1f} deg")
        self.get_logger().info(f"  Speed: {self.motion.MIN_ANGULAR_SPEED}-{self.motion.MAX_ANGULAR_SPEED} rad/s")
        self.get_logger().info(f"  Sweep duration: {self.motion.HALF_SWEEP_DURATION}s")
        self.get_logger().info(f"  Total duration: {self.motion.TOTAL_DURATION}s")

        self.declare_parameter("robots", "Moon")
        robots = self.get_parameter("robots").value
        names = [s.strip() for s in robots.split(",") if s.strip()]

        config = {"Moon": ("Moon", 1), "Basin": ("Basin", -1)}

        self.robots = {}
        scan_topic = self.get_parameter("scan_topic").value
        for name in names:
            ns, direction = config[name]
            self.robots[name] = Phase1Robot(
                name, ns, direction, self,
                self.motion, self.safety, scan_topic
            )
            self.get_logger().info(
                f"  Robot '{name}': ns={ns}, dir={'CW' if direction > 0 else 'CCW'}, "
                f"scan=/{ns}/{scan_topic}"
            )

        self.rotating = False
        self.start_time = None

        # Duty cycle
        self.duty_cycle = self.get_parameter("duty_cycle").value
        self.active_duration = self.get_parameter("active_duration").value
        self.rest_duration = self.get_parameter("rest_duration").value
        self._duty_state = "idle"  # idle, starting, active, resting
        self._duty_timer = 0.0

        self.create_timer(1.0 / self.motion.CONTROL_RATE, self._loop)

        if self.duty_cycle:
            self.get_logger().info(
                f"  Duty cycle: {self.active_duration:.0f}s active / "
                f"{self.rest_duration:.0f}s rest"
            )
            self.create_timer(1.0, self._duty_loop)

        self._auto_start_timer = None
        if self.get_parameter("auto_start").value:
            self._auto_start_timer = self.create_timer(3.0, self._auto_start)

        threading.Thread(target=self._input_loop, daemon=True).start()
        self.get_logger().info("Phase 1 ready. [Enter]=start, s=stop, q=quit")

    def _duty_loop(self):
        """Manages the duty cycle: rest → start lidar → move → stop → rest."""
        now = time.time()

        if self._duty_state == "idle":
            # First cycle: start lidar and begin
            self._duty_state = "starting"
            self._duty_timer = now
            self.get_logger().info("Duty cycle: starting lidar...")
            for r in self.robots.values():
                r.start_lidar()

        elif self._duty_state == "starting":
            # Wait for scan data before moving (max 30s)
            all_ready = all(r._scan_count > 0 for r in self.robots.values())
            if all_ready or (now - self._duty_timer > 30.0):
                self._duty_state = "active"
                self._duty_timer = now
                self.rotating = True
                self.start_time = now
                ready = sum(1 for r in self.robots.values() if r._scan_count > 0)
                self.get_logger().info(
                    f"Duty cycle: ACTIVE ({ready}/{len(self.robots)} robots with scans)"
                )

        elif self._duty_state == "active":
            if now - self._duty_timer >= self.active_duration:
                self._duty_state = "resting"
                self._duty_timer = now
                self.rotating = False
                for r in self.robots.values():
                    r.emergency_stop()
                    r.stop_lidar()
                self.get_logger().info(
                    f"Duty cycle: RESTING for {self.rest_duration:.0f}s (lidar off)"
                )

        elif self._duty_state == "resting":
            if now - self._duty_timer >= self.rest_duration:
                self._duty_state = "starting"
                self._duty_timer = now
                self.get_logger().info("Duty cycle: starting lidar...")
                for r in self.robots.values():
                    r.start_lidar()

    def _auto_start(self):
        self.rotating = True
        self.start_time = time.time()
        self.get_logger().info("Auto-start: rotation started")
        if self._auto_start_timer:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None

    def _loop(self):
        if not self.rotating:
            return
        pass  # No duration limit — duty cycle or manual stop controls lifetime
        dt = 1.0 / self.motion.CONTROL_RATE
        for r in self.robots.values():
            r.update(self.rotating, dt)

    def _input_loop(self):
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
                if cmd == "":
                    self.rotating = True
                    self.start_time = time.time()
                    self.get_logger().info("Manual start")
                elif cmd == "s":
                    self.rotating = False
                    for r in self.robots.values():
                        r.emergency_stop()
                    self.get_logger().info("Manual stop")
                elif cmd == "q":
                    rclpy.shutdown()
                    break
            except EOFError:
                break


def main():
    rclpy.init()
    node = Phase1Controller()

    def _shutdown(sig=None, frame=None):
        node.get_logger().info("Shutting down — stopping all robots")
        for r in node.robots.values():
            r.emergency_stop()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        for r in node.robots.values():
            r.emergency_stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
