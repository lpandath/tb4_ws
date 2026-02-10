#!/usr/bin/env python3
"""
Master Exhibition Controller - Movement Orchestration

Purpose:
--------
Orchestrates synchronized movements for two exhibition robots.
Supports both individual movements and full choreography sequences.

Modes (via parameter):
- "swing_only": Just clock-wise pendulum swings
- "circular_only": Just circular arc motion
- "full_sequence": Swing → Circular → Return to start → Repeat
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


# ============================================================
# Movement Phases
# ============================================================

class MovementPhase(Enum):
    """Global phase for choreography sequencing."""
    IDLE = auto()
    SWING = auto()
    CIRCULAR = auto()
    RETURN_TO_START = auto()
    PAUSED = auto()


class MotionState(Enum):
    """Per-robot motion state within a phase."""
    IDLE = auto()
    ACCELERATING = auto()
    CRUISING = auto()
    DECELERATING = auto()
    PAUSED = auto()
    COMPLETE = auto()


# ============================================================
# Motion Profiles
# ============================================================

@dataclass(frozen=True)
class SwingProfile:
    """Parameters for clock-wise pendulum swing (Movement 1)."""
    linear_speed: float = 0.35
    target_radius: float = 0.8
    rotation_angle: float = math.pi
    
    distance_stop: float = 0.4
    distance_start: float = 0.75
    speed_scale_normal: float = 1.0
    speed_scale_slow: float = 0.5
    speed_scale_stop: float = 0.0
    
    control_rate: float = 50.0
    ramp_fraction: float = 0.15


@dataclass(frozen=True)
class CircularProfile:
    """Parameters for circular arc motion (Movement 2)."""
    linear_speed: float = 0.25
    arc_radius: float = 2.5
    arc_angle: float = math.pi
    
    target_separation: float = 5.0
    
    distance_stop: float = 0.4
    distance_start: float = 0.75
    speed_scale_normal: float = 1.0
    speed_scale_slow: float = 0.5
    speed_scale_stop: float = 0.0
    
    control_rate: float = 50.0
    ramp_fraction: float = 0.15


@dataclass(frozen=True)
class ScheduleProfile:
    """Timing configuration."""
    cycle_period: float = 900.0
    active_window: float = 180.0
    swing_duration: float = 180.0
    circular_duration: float = 180.0
    return_duration: float = 60.0


# ============================================================
# Exhibition Robot
# ============================================================

class ExhibitionRobot:
    """Single robot that can perform multiple movement types."""
    
    def __init__(self, name: str, direction: int, node: Node, 
                 swing_profile: SwingProfile, circular_profile: CircularProfile):
        self.name = name
        self.base_direction = direction
        self.node = node
        self.swing_profile = swing_profile
        self.circular_profile = circular_profile
        
        self._lock = threading.RLock()
        
        self.current_movement = MovementPhase.IDLE
        self.motion_state = MotionState.IDLE
        self.speed_scale = 0.0
        
        self.progress = 0.0
        self.swing_direction = direction
        self.circular_direction = direction
        
        self._obstacle_distance = float('inf')
        self._last_scan_time = time.time()
        
        self.completion_count = 0
        self.pause_count = 0
        self.motion_start_time = None
        
        self._setup_ros()
        node.get_logger().info(f"[{name}] initialized - ready for all movements")
    
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
            
            profile = self.swing_profile
            if self._obstacle_distance > profile.distance_start:
                self.speed_scale = profile.speed_scale_stop
            elif self._obstacle_distance < profile.distance_stop:
                proximity_ratio = self._obstacle_distance / profile.distance_stop
                self.speed_scale = proximity_ratio ** 1.5
            else:
                self.speed_scale = profile.speed_scale_normal
    
    def update(self, movement_phase: MovementPhase, within_window: bool, dt: float):
        """Update robot based on current movement phase."""
        with self._lock:
            return self._update_unsafe(movement_phase, within_window, dt)
    
    def _update_unsafe(self, movement_phase: MovementPhase, within_window: bool, dt: float):
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
        
        if self.current_movement != movement_phase:
            self.current_movement = movement_phase
            self.motion_state = MotionState.IDLE
            self.progress = 0.0
            self.motion_start_time = None
            if movement_phase != MovementPhase.IDLE:
                self.motion_state = MotionState.ACCELERATING
                self.motion_start_time = self.node.get_clock().now()
        
        if movement_phase == MovementPhase.SWING:
            self._update_swing_unsafe(dt)
        elif movement_phase == MovementPhase.CIRCULAR:
            self._update_circular_unsafe(dt)
        elif movement_phase == MovementPhase.RETURN_TO_START:
            self._update_return_unsafe(dt)
        elif movement_phase == MovementPhase.IDLE:
            self._stop_unsafe()
        
        self._publish_velocity_unsafe(movement_phase)
        return True
    
    def _update_swing_unsafe(self, dt: float):
        """Update swing motion (rotation in place)."""
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
        
        profile = self.swing_profile
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
    
    def _update_circular_unsafe(self, dt: float):
        """Update circular arc motion."""
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
        
        profile = self.circular_profile
        linear_speed = profile.linear_speed * self.speed_scale
        angular_speed = linear_speed / profile.arc_radius
        step = abs(angular_speed) * dt
        self.progress = min(self.progress + step, profile.arc_angle)
        
        total = profile.arc_angle
        ramp = profile.ramp_fraction * total
        
        if self.progress >= total:
            self.circular_direction *= -1
            self.progress = 0.0
            self.motion_state = MotionState.ACCELERATING
            self.completion_count += 1
            if self.motion_start_time:
                duration = self.node.get_clock().now() - self.motion_start_time
                sec = duration.nanoseconds * 1e-9
                self.node.get_logger().info(
                    f"[{self.name}] circular arc complete ({sec:.1f}s)"
                )
                self.motion_start_time = self.node.get_clock().now()
        elif self.progress < ramp:
            self.motion_state = MotionState.ACCELERATING
        elif self.progress < total - ramp:
            self.motion_state = MotionState.CRUISING
        else:
            self.motion_state = MotionState.DECELERATING
    
    def _update_return_unsafe(self, dt: float):
        """Move back to starting position."""
        self._update_swing_unsafe(dt)
    
    def _publish_velocity_unsafe(self, movement_phase: MovementPhase):
        """Publish appropriate Twist command for current movement."""
        cmd = Twist()
        
        if self.motion_state in [MotionState.IDLE, MotionState.COMPLETE, MotionState.PAUSED]:
            self.cmd_pub.publish(cmd)
            return
        
        if movement_phase == MovementPhase.SWING:
            profile = self.swing_profile
            angular_speed = (profile.linear_speed / profile.target_radius) * self.speed_scale
            cmd.angular.z = self.swing_direction * angular_speed
        
        elif movement_phase == MovementPhase.CIRCULAR:
            profile = self.circular_profile
            linear_speed = profile.linear_speed * self.speed_scale
            angular_speed = (linear_speed / profile.arc_radius) * self.circular_direction
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
        
        elif movement_phase == MovementPhase.RETURN_TO_START:
            profile = self.swing_profile
            angular_speed = (profile.linear_speed / profile.target_radius) * self.speed_scale
            cmd.angular.z = -self.swing_direction * angular_speed
        
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
                'movement': self.current_movement.name,
                'state': self.motion_state.name,
                'progress': self.progress,
                'distance': self._obstacle_distance,
                'speed_scale': self.speed_scale,
                'completions': self.completion_count,
            }


# ============================================================
# Master Controller
# ============================================================

class MasterExhibitionController(Node):
    """Orchestrates movement sequences for exhibition."""
    
    def __init__(self):
        super().__init__("exhibition_master_controller")
        
        self.declare_parameter("movement_mode", "full_sequence")
        self.declare_parameter("robot_names", ["Moon", "Basin"])
        self.declare_parameter("directions", [-1, 1])
        self.declare_parameter("cycle_period", 900.0)
        self.declare_parameter("active_window", 180.0)
        self.declare_parameter("swing_duration", 180.0)
        self.declare_parameter("circular_duration", 180.0)
        self.declare_parameter("return_duration", 60.0)
        self.declare_parameter("linear_speed", 0.35)
        self.declare_parameter("control_rate", 50.0)
        
        self.movement_mode = self.get_parameter("movement_mode").value
        robot_names = list(self.get_parameter("robot_names").value)
        directions = list(self.get_parameter("directions").value)
        
        swing_profile = SwingProfile(
            linear_speed=float(self.get_parameter("linear_speed").value),
            control_rate=float(self.get_parameter("control_rate").value),
        )
        circular_profile = CircularProfile(
            control_rate=float(self.get_parameter("control_rate").value),
        )
        schedule_profile = ScheduleProfile(
            cycle_period=float(self.get_parameter("cycle_period").value),
            active_window=float(self.get_parameter("active_window").value),
            swing_duration=float(self.get_parameter("swing_duration").value),
            circular_duration=float(self.get_parameter("circular_duration").value),
            return_duration=float(self.get_parameter("return_duration").value),
        )
        
        self.robots: Dict[str, ExhibitionRobot] = {
            name: ExhibitionRobot(name, direction, self, swing_profile, circular_profile)
            for name, direction in zip(robot_names, directions)
        }
        
        self.schedule = schedule_profile
        self.within_active_window = False
        self.cycle_start_time = time.time()
        self.current_movement_phase = MovementPhase.IDLE
        self.phase_start_time = time.time()
        
        self.create_timer(1.0 / swing_profile.control_rate, self._control_loop)
        self.create_timer(1.0, self._schedule_check)
        
        self.get_logger().info("Master Exhibition Controller initialized")
        self.get_logger().info(f"Mode: {self.movement_mode}")
        self._log_config()
    
    def _log_config(self):
        """Log configuration."""
        self.get_logger().info(
            f"Schedule: {self.schedule.cycle_period:.0f}s cycles, "
            f"{self.schedule.active_window:.0f}s active window"
        )
        
        if self.movement_mode == "swing_only":
            self.get_logger().info("Movement: SWING ONLY")
        elif self.movement_mode == "circular_only":
            self.get_logger().info("Movement: CIRCULAR ONLY")
        elif self.movement_mode == "full_sequence":
            self.get_logger().info(
                f"Movement: FULL SEQUENCE - "
                f"Swing {self.schedule.swing_duration:.0f}s + "
                f"Circular {self.schedule.circular_duration:.0f}s + "
                f"Return {self.schedule.return_duration:.0f}s"
            )
    
    def _schedule_check(self):
        """Determine if within active window and update phase."""
        now = time.time()
        elapsed = now - self.cycle_start_time
        
        cycle_pos = elapsed % self.schedule.cycle_period
        self.within_active_window = cycle_pos < self.schedule.active_window
        
        if elapsed >= self.schedule.cycle_period:
            self.cycle_start_time = now
        
        if not self.within_active_window:
            self.current_movement_phase = MovementPhase.IDLE
        else:
            phase_elapsed = cycle_pos
            
            if self.movement_mode == "swing_only":
                self.current_movement_phase = MovementPhase.SWING
            
            elif self.movement_mode == "circular_only":
                self.current_movement_phase = MovementPhase.CIRCULAR
            
            elif self.movement_mode == "full_sequence":
                if phase_elapsed < self.schedule.swing_duration:
                    self.current_movement_phase = MovementPhase.SWING
                elif phase_elapsed < self.schedule.swing_duration + self.schedule.circular_duration:
                    self.current_movement_phase = MovementPhase.CIRCULAR
                elif phase_elapsed < (self.schedule.swing_duration + 
                                     self.schedule.circular_duration + 
                                     self.schedule.return_duration):
                    self.current_movement_phase = MovementPhase.RETURN_TO_START
                else:
                    self.current_movement_phase = MovementPhase.IDLE
    
    def _control_loop(self):
        """Main control loop."""
        dt = 1.0 / self.schedule.cycle_period
        for robot in self.robots.values():
            robot.update(self.current_movement_phase, self.within_active_window, dt)
    
    def emergency_stop_all(self):
        """Stop all robots."""
        for robot in self.robots.values():
            robot.emergency_stop()
        self.get_logger().warn("EMERGENCY STOP triggered")
    
    def get_system_status(self) -> Dict:
        """Get system status."""
        return {
            'mode': self.movement_mode,
            'phase': self.current_movement_phase.name,
            'active': self.within_active_window,
            'robots': [robot.get_status() for robot in self.robots.values()]
        }


# ============================================================
# Entry Point
# ============================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MasterExhibitionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()