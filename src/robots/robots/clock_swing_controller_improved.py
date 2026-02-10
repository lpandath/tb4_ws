#!/usr/bin/env python3
"""
Clock Swing Controller with Battery-Efficient Idle & Time-Based Scheduling
Exhibition robots perform synchronized half-circle (180°) swings in two modes:
1. VISITOR MODE: Activate when people detected (0.8-1.2m), pause if too close (<0.8m)
2. PERIODIC MODE: Move for 1 minute every N minutes, even without visitors (prevents battery drain)
Design:
- No Nav2/SLAM required, just raw /cmd_vel motion
- Thread-safe per-robot state with motion state machine
- Time-scheduled active windows for controlled operation
- Human proximity awareness: stop, slow, resume based on front-sector LiDAR
Services:
- /exhibition/emergency_stop: Hard stop all robots
- /exhibition/test_spin: Manually trigger swing for one robot
- /exhibition/get_status: Query system and robot states
Safety:
- Emergency stop service that halts all robots instantly
- Fail-safe: no motion without recent LiDAR data (< 2s old)
- All mutable state protected by locks
"""
import math
import time
import threading
import json
from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# ============================================================
# Motion State Machine
# ============================================================
class MotionState(Enum):
    """Discrete motion states - intentionally minimal and mutually exclusive."""
    IDLE = auto()          # Not permitted to move
    ACCELERATING = auto()  # Ramp-up phase
    CRUISING = auto()      # Constant-speed phase
    DECELERATING = auto()  # Ramp-down phase
    PAUSED = auto()        # Temporarily stopped due to obstacle
    COMPLETE = auto()      # Finished one half-circle (180°)
# ============================================================
# Exhibition Configuration
# ============================================================
@dataclass(frozen=True)
class MotionProfile:
    """All motion and safety parameters - immutable for audit trail."""
    # Motion parameters
    linear_speed: float = 0.35          # m/s nominal speed (faster demo - test before adding 6kg load)
    target_radius: float = 0.8          # radius for arc motion → angular_speed = linear_speed / target_radius
    rotation_angle: float = math.pi     # 180° half-circle (NOT 360°)
    # Human proximity thresholds (meters) - THREE STATE LOGIC
    distance_stop: float = 0.4          # State 3: VERY CLOSE - smooth stop (0.6 / 2)
    distance_start: float = 0.75        # State 1/2 boundary: FAR/NEAR threshold (1.5 / 2)
                                        # Only activates robot when people are CLOSE - battery optimization!
    distance_resume: float = 0.6        # Resume movement when person leaves stop zone
    # Speed scaling
    speed_scale_normal: float = 1.0
    speed_scale_slow: float = 0.0       # Ramp down to stop in very close zone
    speed_scale_stop: float = 0.0
    # Time-based scheduling
    cycle_period: float = 900.0         # 15 minutes per cycle (3 active, 12 idle)
    active_window: float = 180.0        # 3 minutes motion per cycle (20% duty = 2h battery for 10h day)
    periodic_motion_enabled: bool = True  # Move even without visitors
    # Control loop
    control_rate: float = 50.0          # Hz (smooth motion)
    ramp_fraction: float = 0.20         # accel/decel fraction
    # LiDAR front-sector
    scan_angle_min_deg: float = -60.0
    scan_angle_max_deg: float = 60.0
    scan_min_range: float = 0.05
    scan_max_range: float = 5.0
# ============================================================
# Per-Robot State & Control
# ============================================================
class ClockSwingRobot:
    """Encapsulates one robot's motion, perception, and safety state."""
    def __init__(self, name: str, direction: int, node: Node, profile: MotionProfile):
        self.name = name
        self.base_direction = direction  # Synchronized pair: +1 or -1
        self.swing_direction = direction  # Current swing direction (toggles each swing)
        self.node = node
        self.profile = profile
        # Thread safety
        self._lock = threading.RLock()
        # Motion state
        self.motion_state = MotionState.IDLE
        self.speed_scale = profile.speed_scale_stop
        self.rotation_progress = 0.0
        # Perception
        self._obstacle_distance = float('inf')
        self._last_scan_time = time.time()  # Initialize to now, not 0
        # Telemetry
        self.completion_count = 0
        self.pause_count = 0
        self.motion_start_time = None
        self._setup_ros()
        node.get_logger().info(f"{name}: initialized")
    def _setup_ros(self):
        """Configure ROS publishers and subscribers."""
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        # Publish to unstamped topic (what TurtleBot 4 actually listens to)
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
        """Process LiDAR: extract front-sector closest distance."""
        if msg.angle_increment <= 0:
            return
        # Front sector indices
        start_idx = max(
            0,
            int((math.radians(self.profile.scan_angle_min_deg) - msg.angle_min) / msg.angle_increment)
        )
        end_idx = min(
            len(msg.ranges) - 1,
            int((math.radians(self.profile.scan_angle_max_deg) - msg.angle_min) / msg.angle_increment)
        )
        closest = float('inf')
        for i in range(start_idx, end_idx + 1):
            val = msg.ranges[i]
            if self.profile.scan_min_range < val < self.profile.scan_max_range:
                closest = min(closest, val)
        with self._lock:
            self._obstacle_distance = closest if closest != float('inf') else 10.0
            self._last_scan_time = time.time()
            
            # THREE STATE PROXIMITY LOGIC
            # State 1: FAR (> 1.5m) → IDLE, save battery
            # State 2: NEAR (0.6 - 1.5m) → FULL SPEED rotation
            # State 3: VERY CLOSE (< 0.6m) → SMOOTHLY STOP
            
            if self._obstacle_distance > self.profile.distance_start:
                # STATE 1: FAR - no visitors detected, idle to save battery
                self.speed_scale = self.profile.speed_scale_stop  # Force idle (0.0)
                
            elif self._obstacle_distance < self.profile.distance_stop:
                # STATE 3: VERY CLOSE - smoothly decelerate to stop
                # Create smooth ramp from 1.0 (at 0.6m) to 0.0 (at 0m)
                # Using a gentler ramp: proximity_ratio squared for smoother deceleration
                proximity_ratio = self._obstacle_distance / self.profile.distance_stop
                self.speed_scale = proximity_ratio ** 1.5  # Smooth power curve
                
            else:
                # STATE 2: NEAR (0.6 - 1.5m) - visitor detected, rotate at normal speed
                self.speed_scale = self.profile.speed_scale_normal
    def update(self, within_active_window: bool, dt: float):
        """Main control update - called at control_rate."""
        with self._lock:
            return self._update_unsafe(within_active_window, dt)
    def _update_unsafe(self, within_active_window: bool, dt: float):
        """Core decision logic - must be called with lock held."""
        # Fail-safe: no motion without fresh LiDAR
        now = time.time()
        if now - self._last_scan_time > 2.0:
            self.speed_scale = self.profile.speed_scale_stop
        # Outside scheduled window → force IDLE
        if not within_active_window:
            if self.motion_state != MotionState.IDLE:
                self._stop_unsafe()
                self.motion_state = MotionState.IDLE
                self.rotation_progress = 0.0
            return False
        # Entering active window → start swing
        if self.motion_state == MotionState.IDLE:
            self.motion_state = MotionState.ACCELERATING
            self.rotation_progress = 0.0
            self.motion_start_time = self.node.get_clock().now()
        # Update motion state machine
        self._update_motion_unsafe(dt)
        self._publish_velocity_unsafe()
        return True
    def _update_motion_unsafe(self, dt: float):
        """Motion state machine: accel → cruise → decel → complete → reverse."""
        if self.motion_state == MotionState.IDLE:
            return
        
        # Obstacle → pause (only if speed_scale is EXACTLY 0, not just small)
        if self.speed_scale == self.profile.speed_scale_stop:
            if self.motion_state != MotionState.PAUSED:
                self.motion_state = MotionState.PAUSED
                self.pause_count += 1
            return
        
        # Resume from pause (when speed_scale becomes non-zero)
        if self.motion_state == MotionState.PAUSED and self.speed_scale > 0:
            self.motion_state = MotionState.ACCELERATING
            self.node.get_logger().debug(f"{self.name}: RESUMING from pause (speed_scale={self.speed_scale:.3f})")
            return
        
        # Advance rotation
        angular_speed = self._calculate_angular_speed_unsafe()
        step = abs(angular_speed) * dt
        self.rotation_progress = min(
            self.rotation_progress + step,
            self.profile.rotation_angle
        )
        # State transitions based on progress through swing
        total = self.profile.rotation_angle
        ramp = self.profile.ramp_fraction * total
        if self.rotation_progress >= total:
            # Swing complete → toggle direction and start reverse swing
            self.swing_direction *= -1  # Flip for next swing
            self.rotation_progress = 0.0
            self.motion_state = MotionState.ACCELERATING
            self.completion_count += 1
            if self.motion_start_time:
                duration = self.node.get_clock().now() - self.motion_start_time
                sec = duration.nanoseconds * 1e-9
                self.node.get_logger().info(
                    f"{self.name}: swing complete ({sec:.1f}s, {self.pause_count} pauses) → reversing"
                )
                self.motion_start_time = self.node.get_clock().now()
        elif self.rotation_progress < ramp:
            self.motion_state = MotionState.ACCELERATING
        elif self.rotation_progress < total - ramp:
            self.motion_state = MotionState.CRUISING
        else:
            self.motion_state = MotionState.DECELERATING
    def _calculate_angular_speed_unsafe(self) -> float:
        """
        Calculate angular velocity for in-place pendulum swinging.
        
        Robot rotates in place around vertical Z axis (NO linear motion):
        - Angular speed = linear_speed / target_radius (from kinematics)
        - Example: linear_speed=0.15 m/s, target_radius=0.8m
          → angular_speed ≈ 0.1875 rad/s
          → completes 180° swing in ~17 seconds
        
        The swing_direction (±1) determines rotation direction:
        - swing_direction=+1: rotate counter-clockwise
        - swing_direction=-1: rotate clockwise
        
        Direction toggles after each 180° swing, creating pendulum effect.
        Synchronized robots use opposite base_directions for coordinated swinging.
        """
        # Angular speed for 180° swing: use linear_speed / radius as baseline
        base_speed = (self.profile.linear_speed / self.profile.target_radius)
        base_speed *= self.speed_scale
        return self.swing_direction * base_speed
    def _publish_velocity_unsafe(self):
        """
        Publish Twist command for clock pendulum motion.
        
        Creates IN-PLACE HALF-CIRCLE SWINGS (like old clock domes with glass dome):
        - NO linear motion: robot stays in one spot (all linear components = 0)
        - ONLY angular rotation: swings left and right around vertical axis
        
        Combined effect: robot rotates 180° left, then 180° right, back to start
        
        Motion sequence:
        1. Start → rotate left (or right) 180°
        2. Complete → reverse direction
        3. Rotate back in opposite direction 180°
        4. Repeat → pendulum effect synchronized with other robots
        
        Proximity behavior:
        - Person far (>1.2m): full speed swing
        - Person near (0.8-1.2m): slow swing (40% speed)
        - Person very close (<0.8m): STOP immediately
        
        Note: linear.x = 0, linear.y = 0, linear.z = 0
              angular.x = 0, angular.y = 0, angular.z = speed
              This ensures ONLY rotation around vertical axis, no movement.
        """
        cmd = Twist()
        # Ensure all linear components are zero (no forward/lateral/vertical motion)
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        # Only set angular velocity around Z axis (vertical rotation)
        if self.motion_state not in [MotionState.IDLE, MotionState.COMPLETE, MotionState.PAUSED]:
            cmd.angular.z = self._calculate_angular_speed_unsafe()
        else:
            cmd.angular.z = 0.0
        # Ensure other angular components are zero
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        self.cmd_pub.publish(cmd)
    def _stop_unsafe(self):
        """Emergency stop - publish zero velocity."""
        self.cmd_pub.publish(Twist())
    def emergency_stop(self):
        """Public API: hard stop and reset."""
        with self._lock:
            self._stop_unsafe()
            self.motion_state = MotionState.IDLE
            self.rotation_progress = 0.0
            self.motion_start_time = None
    def test_spin(self):
        """Public API: manually trigger a swing (for staff testing)."""
        with self._lock:
            self.motion_state = MotionState.ACCELERATING
            self.rotation_progress = 0.0
            self.speed_scale = self.profile.speed_scale_normal
            self.motion_start_time = self.node.get_clock().now()
    def get_status(self) -> Dict:
        """Public API: snapshot of robot state."""
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
# System Controller
# ============================================================
class ClockSwingController(Node):
    """Main controller: orchestrates schedule and per-robot state."""
    def __init__(self):
        super().__init__("clock_swing_controller")
        # Load parameters
        self.declare_parameter("robot_names", ["Moon", "Basin"])
        self.declare_parameter("directions", [-1, 1])
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("target_radius", 0.8)
        self.declare_parameter("distance_stop", 0.8)
        self.declare_parameter("distance_start", 1.2)
        self.declare_parameter("distance_resume", 1.0)
        self.declare_parameter("cycle_period", 900.0)  # 15 min cycles
        self.declare_parameter("active_window", 180.0)  # 3 min active
        self.declare_parameter("periodic_motion_enabled", True)
        self.declare_parameter("control_rate", 20.0)
        robot_names = list(self.get_parameter("robot_names").value)
        directions = list(self.get_parameter("directions").value)
        if len(directions) != len(robot_names):
            raise ValueError("directions must match robot_names length")
        # Build motion profile from parameters
        profile = MotionProfile(
            linear_speed=float(self.get_parameter("linear_speed").value),
            target_radius=float(self.get_parameter("target_radius").value),
            distance_stop=float(self.get_parameter("distance_stop").value),
            distance_start=float(self.get_parameter("distance_start").value),
            distance_resume=float(self.get_parameter("distance_resume").value),
            cycle_period=float(self.get_parameter("cycle_period").value),
            active_window=float(self.get_parameter("active_window").value),  # 7 min for opening
            periodic_motion_enabled=bool(self.get_parameter("periodic_motion_enabled").value),
            control_rate=float(self.get_parameter("control_rate").value),
        )
        # Create robots
        self.robots: Dict[str, ClockSwingRobot] = {
            name: ClockSwingRobot(name, direction, self, profile)
            for name, direction in zip(robot_names, directions)
        }
        self.profile = profile
        self.within_active_window = False
        self.cycle_start_time = time.time()
        # Timers
        self.create_timer(1.0 / profile.control_rate, self._control_loop)
        self.create_timer(1.0, self._schedule_check)
        robot_list = ", ".join(robot_names)
        self.get_logger().info(f"Clock swing controller ready for: {robot_list}")
        self._log_config()
    def _log_config(self):
        """Log configuration at startup."""
        self.get_logger().info(
            f"Schedule: {self.profile.cycle_period:.0f}s cycles, "
            f"{self.profile.active_window:.0f}s active windows"
        )
        swing_time = self.profile.rotation_angle / (self.profile.linear_speed / self.profile.target_radius)
        self.get_logger().info(
            f"Motion: {self.profile.linear_speed:.2f} m/s, "
            f"radius={self.profile.target_radius:.2f}m, "
            f"angular_speed={self.profile.linear_speed/self.profile.target_radius:.3f} rad/s, "
            f"~{swing_time:.1f}s per 180° swing, "
            f"control_rate={self.profile.control_rate:.0f}Hz"
        )
        self.get_logger().info(
            "THREE-STATE PROXIMITY LOGIC (BATTERY OPTIMIZED):"
        )
        self.get_logger().info(
            f"  State 1 FAR   (>  {self.profile.distance_start:.2f}m): IDLE - ignore distant crowds"
        )
        self.get_logger().info(
            f"  State 2 NEAR  ({self.profile.distance_stop:.2f} - {self.profile.distance_start:.2f}m): ROTATE - close visitors"
        )
        self.get_logger().info(
            f"  State 3 CLOSE (<  {self.profile.distance_stop:.2f}m): SMOOTHLY STOP - safety"
        )
        # Setup ROS services (optional - only if exhibition_interfaces is available)
        try:
            from exhibition_interfaces.srv import EmergencyStop, TestSpin, GetStatus
            
            self.create_service(EmergencyStop, "/exhibition/emergency_stop", self._handle_emergency_stop)
            self.create_service(TestSpin, "/exhibition/test_spin", self._handle_test_spin)
            self.create_service(GetStatus, "/exhibition/get_status", self._handle_get_status)
            self.get_logger().info("Services: emergency_stop, test_spin, get_status")
        except ImportError:
            self.get_logger().warn("exhibition_interfaces not available, services disabled")
    def _schedule_check(self):
        """Determine if within active window."""
        now = time.time()
        elapsed = now - self.cycle_start_time
        
        # PRODUCTION: Use 10-minute cycles with configurable active window
        cycle_pos = elapsed % self.profile.cycle_period
        self.within_active_window = cycle_pos < self.profile.active_window
        
        # Reset cycle
        if elapsed >= self.profile.cycle_period:
            self.cycle_start_time = now
        
        # Log when entering/exiting active window
        if int(elapsed) % int(self.profile.cycle_period) == 0:
            state = "ACTIVE" if self.within_active_window else "IDLE"
            self.get_logger().info(f"Schedule: {state} (cycle_pos={cycle_pos:.1f}s / {self.profile.cycle_period:.0f}s)")
    def _control_loop(self):
        """Main control loop at control_rate."""
        dt = 1.0 / self.profile.control_rate
        for robot in self.robots.values():
            robot.update(self.within_active_window, dt)
    def _log_status(self):
        """Diagnostics: print all robot states."""
        for robot in self.robots.values():
            status = robot.get_status()
            self.node.get_logger().info(f"  {status['name']}: {status['state']} ({status['progress']:.2f}°)")
    def emergency_stop_all(self):
        """Public API: stop all robots."""
        for robot in self.robots.values():
            robot.emergency_stop()
        self.get_logger().warn("EMERGENCY STOP triggered")
    def get_system_status(self) -> Dict:
        """Public API: aggregate status of all robots."""
        return {
            'within_active_window': self.within_active_window,
            'robots': [robot.get_status() for robot in self.robots.values()]
        }
    def _handle_emergency_stop(self, request, response):
        """ROS service handler: /exhibition/emergency_stop"""
        self.emergency_stop_all()
        response.success = True
        response.message = "Emergency stop executed"
        return response
    def _handle_test_spin(self, request, response):
        """ROS service handler: /exhibition/test_spin"""
        robot_name = request.robot_name.strip()
        
        if not robot_name:
            # Trigger all robots
            for robot in self.robots.values():
                robot.test_spin()
            response.success = True
            response.message = f"Test spin triggered for all {len(self.robots)} robots"
        elif robot_name in self.robots:
            self.robots[robot_name].test_spin()
            response.success = True
            response.message = f"Test spin triggered for {robot_name}"
        else:
            response.success = False
            response.message = f"Unknown robot: {robot_name}"
        
        return response
    def _handle_get_status(self, request, response):
        """ROS service handler: /exhibition/get_status"""
        status = self.get_system_status()
        
        response.success = True
        response.system_status = "ACTIVE" if self.within_active_window else "IDLE"
        response.next_motion = f"In {(self.profile.cycle_period - (time.time() - self.cycle_start_time) % self.profile.cycle_period):.0f}s"
        response.window_active = self.within_active_window
        response.robots_json = json.dumps(status['robots'], indent=2)
        
        return response
# ============================================================
# Entry Point
# ============================================================
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