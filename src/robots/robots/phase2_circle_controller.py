#!/usr/bin/env python3
"""
PHASE 2: CIRCLE CONTROLLER (MAP FRAME)
One node drives one or two robots on a circle in the shared map frame.

- Single robot: follow circle (center + radius) in map.
- Two robots: same circle, maintain target relative angle (e.g. 180° opposite).
"""

import math
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class Phase2State(Enum):
    IDLE = auto()
    CIRCLING = auto()


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation around z) from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    """Signed difference (a - b) in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Phase2CircleController(Node):
    """
    Drives one or two robots on a circle in map frame.
    All poses and circle/waypoints are in the same map frame.
    """

    def __init__(self):
        super().__init__("phase2_circle_controller")

        # ----- Parameters -----
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robots", "Moon")  # string: "Moon" or "Moon,Basin" (so launch robots:=Moon works)
        self.declare_parameter("circle_center_x", 0.0)
        self.declare_parameter("circle_center_y", 0.0)
        self.declare_parameter("circle_radius", 1.0)
        self.declare_parameter("circle_direction", 1)  # 1 = CCW, -1 = CW
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("control_rate", 30.0)
        self.declare_parameter("target_relative_angle_rad", 3.141592653589793)  # 180° for opposite
        self.declare_parameter("k_heading", 1.0)
        self.declare_parameter("k_radius", 0.2)
        self.declare_parameter("radius_deadband", 0.15)   # don't correct speed if within this (m) of target radius
        self.declare_parameter("max_accel_linear", 0.06)   # max change in v per control step (smoother)
        self.declare_parameter("max_accel_angular", 0.12) # max change in omega per control step
        self.declare_parameter("start_immediately", False)

        self.map_frame = self.get_parameter("map_frame").value
        robots_param = self.get_parameter("robots").value
        if isinstance(robots_param, str):
            self.robot_names = [s.strip() for s in robots_param.split(",") if s.strip()]
        else:
            self.robot_names = list(robots_param) if robots_param else ["Moon"]

        self.cx = float(self.get_parameter("circle_center_x").value)
        self.cy = float(self.get_parameter("circle_center_y").value)
        self.radius = float(self.get_parameter("circle_radius").value)
        self.direction = int(self.get_parameter("circle_direction").value)
        if self.direction not in (1, -1):
            self.direction = 1
        self.linear_speed = self.get_parameter("linear_speed").value
        self.control_rate = self.get_parameter("control_rate").value
        self.target_relative_angle = self.get_parameter("target_relative_angle_rad").value
        self.k_heading = self.get_parameter("k_heading").value
        self.k_radius = self.get_parameter("k_radius").value
        self.radius_deadband = self.get_parameter("radius_deadband").value
        self.max_accel_linear = self.get_parameter("max_accel_linear").value
        self.max_accel_angular = self.get_parameter("max_accel_angular").value
        self.start_immediately = self.get_parameter("start_immediately").value

        # ----- State -----
        self.state = Phase2State.IDLE
        self.reference_angle = 0.0  # for two-robot sync (leader angle)
        self._last_tf_warn_time = 0.0
        self._circling_logged = False
        # Smoothed commands (per robot) to avoid jumping
        self._last_v: dict = {}
        self._last_omega: dict = {}
        self._max_dv = self.max_accel_linear
        self._max_domega = self.max_accel_angular

        # ----- TF -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- Cmd_vel publishers (one per robot) -----
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1)
        self.cmd_pubs = {}
        for name in self.robot_names:
            self.cmd_pubs[name] = self.create_publisher(
                Twist, f"/{name}/cmd_vel_unstamped", qos
            )

        # ----- Control timer -----
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_callback
        )

        if self.start_immediately:
            self.state = Phase2State.CIRCLING
            self.get_logger().info("Phase 2 started (start_immediately=true)")

        self.get_logger().info("=" * 60)
        self.get_logger().info("PHASE 2: CIRCLE CONTROLLER (map frame)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Map frame: {self.map_frame}")
        self.get_logger().info(f"Robots: {self.robot_names}")
        self.get_logger().info(
            f"Circle: center=({self.cx:.3f}, {self.cy:.3f}) m, R={self.radius:.3f} m, "
            f"direction={'CCW' if self.direction == 1 else 'CW'}"
        )
        self.get_logger().info(
            "  ^ If the robot drives a different size circle, check this line matches your launch args."
        )
        self.get_logger().info("Start: set start_immediately:=true or call /phase2_circle_controller/start")
        self.get_logger().info("=" * 60)

        # Service to start (optional)
        from std_srvs.srv import Trigger
        self.start_srv = self.create_service(
            Trigger, "phase2_circle_controller/start", self._start_callback
        )

    def _start_callback(self, request, response):
        if self.state == Phase2State.IDLE:
            self.state = Phase2State.CIRCLING
            self.get_logger().info("Phase 2 started via service.")
            response.success = True
            response.message = "Phase 2 started."
        else:
            response.success = False
            response.message = "Already running."
        return response

    def _get_pose_in_map(self, base_frame: str) -> Optional[Tuple[float, float, float]]:
        """Return (x, y, yaw) in map frame, or None if transform not available."""
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                base_frame,
                when,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            yaw = quaternion_to_yaw(r.x, r.y, r.z, r.w)
            return (t.x, t.y, yaw)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _stop_robot(self, name: str) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pubs[name].publish(cmd)

    def _publish_cmd(self, name: str, linear_x: float, angular_z: float) -> None:
        # Rate-limit changes for smoother motion (avoid jumping)
        last_v = self._last_v.get(name, linear_x)
        last_omega = self._last_omega.get(name, angular_z)
        dv = max(-self._max_dv, min(self._max_dv, linear_x - last_v))
        domega = max(-self._max_domega, min(self._max_domega, angular_z - last_omega))
        linear_x = last_v + dv
        angular_z = last_omega + domega
        self._last_v[name] = linear_x
        self._last_omega[name] = angular_z
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(angular_z)
        self.cmd_pubs[name].publish(cmd)

    def _control_circle_single(
        self, x: float, y: float, theta: float, robot_name: str
    ) -> None:
        """One robot: follow circle. Tangent heading + radius correction."""
        dx = x - self.cx
        dy = y - self.cy
        dist_to_center = math.sqrt(dx * dx + dy * dy)
        if dist_to_center < 1e-6:
            self._stop_robot(robot_name)
            return
        phi = math.atan2(dy, dx)
        radius_error = dist_to_center - self.radius
        # Desired heading = tangent to circle (CCW: phi + 90°, CW: phi - 90°)
        tangent_heading = phi + (math.pi / 2.0) * self.direction
        heading_error = angle_diff(tangent_heading, theta)
        omega_ff = self.direction * (self.linear_speed / max(self.radius, 0.3))
        omega = omega_ff + self.k_heading * heading_error
        # Speed: only correct if outside deadband so we don't shrink the circle
        if abs(radius_error) <= self.radius_deadband:
            v = self.linear_speed
        else:
            v = self.linear_speed + self.k_radius * radius_error
        v = max(self.linear_speed * 0.25, min(self.linear_speed * 1.2, v))
        self._publish_cmd(robot_name, v, omega)

    def _control_circle_dual(
        self,
        poses: List[Tuple[float, float, float]],
    ) -> None:
        """
        Two robots: both on circle, maintain target_relative_angle.
        Robot 0 is reference; robot 1 target angle = reference + target_relative_angle.
        """
        if len(poses) < 2 or len(self.robot_names) < 2:
            if len(poses) == 1:
                self._control_circle_single(
                    poses[0][0], poses[0][1], poses[0][2], self.robot_names[0]
                )
            return

        x0, y0, theta0 = poses[0]
        x1, y1, theta1 = poses[1]

        # Update reference angle from leader (robot 0) position on circle
        dx0 = x0 - self.cx
        dy0 = y0 - self.cy
        self.reference_angle = math.atan2(dy0, dx0)

        # Target angles: robot0 at reference, robot1 at reference + target_relative_angle
        target_phi0 = self.reference_angle
        target_phi1 = self.reference_angle + self.target_relative_angle

        for i, (x, y, theta) in enumerate(poses):
            name = self.robot_names[i]
            dx = x - self.cx
            dy = y - self.cy
            dist_to_center = math.sqrt(dx * dx + dy * dy)
            if dist_to_center < 1e-6:
                self._stop_robot(name)
                continue
            phi = math.atan2(dy, dx)
            target_phi = target_phi0 if i == 0 else target_phi1
            # Desired tangent = target_phi + 90° * direction (we want to move toward target_phi)
            tangent_heading = target_phi + (math.pi / 2.0) * self.direction
            heading_error = angle_diff(tangent_heading, theta)
            radius_error = dist_to_center - self.radius
            omega_ff = self.direction * (self.linear_speed / max(self.radius, 0.3))
            omega = omega_ff + self.k_heading * heading_error
            if abs(radius_error) <= self.radius_deadband:
                v = self.linear_speed
            else:
                v = self.linear_speed + self.k_radius * radius_error
            v = max(self.linear_speed * 0.25, min(self.linear_speed * 1.2, v))
            self._publish_cmd(name, v, omega)

    def control_callback(self) -> None:
        if self.state == Phase2State.IDLE:
            for name in self.robot_names:
                self._stop_robot(name)
            return

        # Get poses for all robots (in map frame)
        base_frames = [f"{name}/base_link" for name in self.robot_names]
        poses: List[Optional[Tuple[float, float, float]]] = []
        for name, bf in zip(self.robot_names, base_frames):
            pose = self._get_pose_in_map(bf)
            poses.append(pose)

        # If any pose missing, stop and wait (TF not available)
        if any(p is None for p in poses):
            for name in self.robot_names:
                self._stop_robot(name)
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_tf_warn_time >= 2.0:
                self._last_tf_warn_time = now
                missing = [bf for bf, p in zip(base_frames, poses) if p is None]
                self.get_logger().warn(
                    f"Phase 2: no TF from '{self.map_frame}' to robot base_link. "
                    f"Missing: {missing}. Is localization running? Check: "
                    f"ros2 run tf2_ros tf2_echo {self.map_frame} {base_frames[0]}"
                )
            return

        if self.state == Phase2State.CIRCLING:
            if not self._circling_logged:
                self._circling_logged = True
                self.get_logger().info("Phase 2: CIRCLING - sending cmd_vel")
            if len(poses) == 1:
                self._control_circle_single(
                    poses[0][0], poses[0][1], poses[0][2], self.robot_names[0]
                )
            else:
                self._control_circle_dual(poses)


def main(args=None):
    rclpy.init(args=args)
    node = Phase2CircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for name in node.robot_names:
            node._stop_robot(name)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()