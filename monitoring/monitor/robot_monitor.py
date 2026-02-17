#!/usr/bin/env python3
"""
Robot Monitor — shared multi-robot monitoring for TurtleBot4
=============================================================
Single RobotMonitor(Node) class that handles one or more robots.
Each robot gets its own subscriptions, buffers, and SSH/ping workers.

All InfluxDB points are tagged with robot=<name> for Grafana filtering.

Fixes vs the original basin_monitor.py:
  - SSH + ping + iwconfig run in daemon threads (non-blocking executor)
  - No /scan subscription (biggest WiFi waste, ~3KB/msg)
  - SSHPASS password via env var (-e flag) instead of command-line arg
  - destroy_node() before rclpy.shutdown()
  - IMU buffer uses tuples instead of dicts
  - Dropped global topics (diagnostics, rosout) — low value for production
"""

import math
import os
import signal
import subprocess
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String

from irobot_create_msgs.msg import (
    DockStatus,
    HazardDetectionVector,
    KidnapStatus,
    StopStatus,
    WheelStatus,
)

from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import WriteOptions

# ---------------------------------------------------------------------------
# Configuration from environment
# ---------------------------------------------------------------------------
INFLUXDB_URL = os.environ.get("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN = os.environ.get("INFLUXDB_TOKEN", "basin-monitor-token-2026")
INFLUXDB_ORG = os.environ.get("INFLUXDB_ORG", "turtlebot")
INFLUXDB_BUCKET = os.environ.get("INFLUXDB_BUCKET", "exhibition")

CONSOLE_INTERVAL = float(os.environ.get("CONSOLE_INTERVAL", "30"))
SSH_POLL_INTERVAL = float(os.environ.get("SSH_POLL_INTERVAL", "30"))
NETWORK_CHECK_INTERVAL = float(os.environ.get("NETWORK_CHECK_INTERVAL", "5"))

SSH_USER = os.environ.get("SSH_USER", "ubuntu")
SSH_PASSWORD = os.environ.get("SSH_PASSWORD", "turtlebot4")


# ---------------------------------------------------------------------------
# Robot configuration
# ---------------------------------------------------------------------------
@dataclass
class RobotConfig:
    name: str        # "Basin" / "Moon"
    ip: str          # "192.168.1.158"
    namespace: str   # "Basin" (no leading slash)
    ssh_user: str = ""
    ssh_password: str = ""

    def __post_init__(self):
        self.namespace = self.namespace.strip("/")
        if not self.ssh_user:
            self.ssh_user = SSH_USER
        if not self.ssh_password:
            self.ssh_password = SSH_PASSWORD


# ---------------------------------------------------------------------------
# Per-robot state
# ---------------------------------------------------------------------------
class _RobotState:
    """All mutable state for a single robot."""

    def __init__(self, config: RobotConfig):
        self.config = config

        # Topic stats
        self.stats: dict[str, _TopicStats] = {}

        # Console summary values
        self.battery_pct = None
        self.battery_voltage = None
        self.battery_current = None
        self.inferred_state = "UNKNOWN"
        self.ping_ms = None
        self.wifi_dbm = None
        self.is_docked = None

        # Buffers for high-rate topics
        self.cmd_vel_buf: list[tuple[float, float]] = []
        self.odom_latest = None
        self.imu_buf: list[tuple[float, float, float, float, float, float, float, float]] = []

        # On-change detection
        self.last_dock_status = None
        self.last_kidnap = None
        self.last_stop = None
        self.last_ip = None
        self.last_wheel_time = 0.0


# ---------------------------------------------------------------------------
# Topic rate tracker
# ---------------------------------------------------------------------------
class _TopicStats:
    """Track message rates and gaps for a single topic."""

    def __init__(self, window_sec=60.0):
        self._times: deque[float] = deque()
        self._window = window_sec
        self.last_time = 0.0
        self.msg_count = 0

    def record(self):
        now = time.time()
        self._times.append(now)
        self.last_time = now
        self.msg_count += 1
        cutoff = now - self._window
        while self._times and self._times[0] < cutoff:
            self._times.popleft()

    def rate(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0:
            return 0.0
        return (len(self._times) - 1) / span

    def gap(self) -> float:
        if self.last_time == 0:
            return float("inf")
        return time.time() - self.last_time


# ---------------------------------------------------------------------------
# Main monitor node
# ---------------------------------------------------------------------------
class RobotMonitor(Node):
    """
    Multi-robot ROS2 monitor node.

    Uses a SingleThreadedExecutor (the rclpy.spin default), so all callbacks
    run sequentially on the main thread. SSH/ping are offloaded to daemon
    threads to avoid blocking the executor.
    """

    def __init__(self, robots: list[RobotConfig]):
        super().__init__("robot_monitor")

        self._robots: dict[str, _RobotState] = {}

        # InfluxDB writer with batching (thread-safe for daemon threads)
        self._influx = InfluxDBClient(
            url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG
        )
        self._write_api = self._influx.write_api(
            write_options=WriteOptions(
                batch_size=1000,
                flush_interval=5_000,
                jitter_interval=0,
                retry_interval=5_000,
            )
        )

        # Set up each robot
        for cfg in robots:
            rs = _RobotState(cfg)
            self._robots[cfg.name] = rs
            self._setup_robot_subscriptions(rs)
            self.get_logger().info(
                f"Robot '{cfg.name}' configured — NS=/{cfg.namespace}, IP={cfg.ip}"
            )

        self._setup_timers()
        self.get_logger().info("Subscriptions and timers ready")

    # -------------------------------------------------------------------
    # Subscriptions (10 per robot)
    # -------------------------------------------------------------------
    def _setup_robot_subscriptions(self, rs: _RobotState):
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        ns = rs.config.namespace

        def sub(topic_suffix, msg_type, callback, qos):
            topic = f"/{ns}/{topic_suffix}"
            name = topic_suffix.split("/")[-1]
            rs.stats[name] = _TopicStats()
            self.create_subscription(msg_type, topic, callback, qos)

        sub("battery_state", BatteryState, lambda msg: self._on_battery(rs, msg), qos_sensor)
        sub("cmd_vel_unstamped", Twist, lambda msg: self._on_cmd_vel(rs, msg), qos_reliable)
        sub("odom", Odometry, lambda msg: self._on_odom(rs, msg), qos_sensor)
        sub("imu", Imu, lambda msg: self._on_imu(rs, msg), qos_sensor)
        sub("hazard_detection", HazardDetectionVector, lambda msg: self._on_hazard(rs, msg), qos_sensor)
        sub("dock_status", DockStatus, lambda msg: self._on_dock(rs, msg), qos_sensor)
        sub("kidnap_status", KidnapStatus, lambda msg: self._on_kidnap(rs, msg), qos_sensor)
        sub("stop_status", StopStatus, lambda msg: self._on_stop(rs, msg), qos_sensor)
        sub("wheel_status", WheelStatus, lambda msg: self._on_wheel(rs, msg), qos_sensor)
        sub("ip", String, lambda msg: self._on_ip(rs, msg), qos_reliable)

    # -------------------------------------------------------------------
    # InfluxDB write helper — tags every point with robot name
    # -------------------------------------------------------------------
    def _write(self, rs: _RobotState, point: Point):
        try:
            point = point.tag("robot", rs.config.name)
            self._write_api.write(bucket=INFLUXDB_BUCKET, record=point)
        except Exception as e:
            self.get_logger().warn(
                f"InfluxDB write error ({rs.config.name}): {e}",
                throttle_duration_sec=10.0,
            )

    # -------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------
    def _on_battery(self, rs: _RobotState, msg: BatteryState):
        rs.stats["battery_state"].record()
        pct = msg.percentage
        if pct <= 1.0:
            pct *= 100.0
        rs.battery_pct = pct
        rs.battery_voltage = msg.voltage
        rs.battery_current = msg.current

        p = (
            Point("battery")
            .field("voltage", float(msg.voltage))
            .field("percentage", float(pct))
            .field("current", float(msg.current))
            .field("temperature", float(msg.temperature))
        )
        if not math.isnan(msg.charge):
            p = p.field("charge", float(msg.charge))
        if not math.isnan(msg.capacity):
            p = p.field("capacity", float(msg.capacity))
        p = p.field("power_supply_status", int(msg.power_supply_status))
        self._write(rs, p)

    def _on_cmd_vel(self, rs: _RobotState, msg: Twist):
        rs.stats["cmd_vel_unstamped"].record()
        rs.cmd_vel_buf.append((msg.linear.x, msg.angular.z))

    def _on_odom(self, rs: _RobotState, msg: Odometry):
        rs.stats["odom"].record()
        rs.odom_latest = msg

    def _on_imu(self, rs: _RobotState, msg: Imu):
        rs.stats["imu"].record()
        rs.imu_buf.append((
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ))

    def _on_hazard(self, rs: _RobotState, msg: HazardDetectionVector):
        rs.stats["hazard_detection"].record()
        for det in msg.detections:
            p = (
                Point("hazards")
                .field("hazard_type", int(det.type))
                .field("description", det.header.frame_id)
            )
            self._write(rs, p)

    def _on_dock(self, rs: _RobotState, msg: DockStatus):
        rs.stats["dock_status"].record()
        val = (msg.is_docked, msg.dock_visible)
        if val == rs.last_dock_status:
            return
        rs.last_dock_status = val
        rs.is_docked = msg.is_docked

        p = (
            Point("dock")
            .field("is_docked", msg.is_docked)
            .field("dock_visible", msg.dock_visible)
        )
        self._write(rs, p)

    def _on_kidnap(self, rs: _RobotState, msg: KidnapStatus):
        rs.stats["kidnap_status"].record()
        val = msg.is_kidnapped
        if val == rs.last_kidnap:
            return
        rs.last_kidnap = val

        p = (
            Point("events")
            .tag("source", "kidnap")
            .field("value", val)
            .field("description", "kidnapped" if val else "restored")
        )
        self._write(rs, p)

    def _on_stop(self, rs: _RobotState, msg: StopStatus):
        rs.stats["stop_status"].record()
        val = msg.is_stopped
        if val == rs.last_stop:
            return
        rs.last_stop = val

        p = (
            Point("events")
            .tag("source", "stop")
            .field("value", val)
            .field("description", "stopped" if val else "released")
        )
        self._write(rs, p)

    def _on_wheel(self, rs: _RobotState, msg: WheelStatus):
        rs.stats["wheel_status"].record()
        now = time.time()
        if now - rs.last_wheel_time < 1.0:
            return
        rs.last_wheel_time = now

        p = (
            Point("wheels")
            .field("current_ma_left", int(msg.current_ma_left))
            .field("current_ma_right", int(msg.current_ma_right))
            .field("pwm_left", int(msg.pwm_left))
            .field("pwm_right", int(msg.pwm_right))
        )
        self._write(rs, p)

    def _on_ip(self, rs: _RobotState, msg: String):
        rs.stats["ip"].record()
        val = msg.data
        if val == rs.last_ip:
            return
        rs.last_ip = val

        p = (
            Point("events")
            .tag("source", "ip")
            .field("value", True)
            .field("description", val)
        )
        self._write(rs, p)

    # -------------------------------------------------------------------
    # Timers
    # -------------------------------------------------------------------
    def _setup_timers(self):
        self.create_timer(1.0, self._flush_summaries_all)
        self.create_timer(NETWORK_CHECK_INTERVAL, self._check_network_all)
        self.create_timer(SSH_POLL_INTERVAL, self._poll_system_all)
        self.create_timer(CONSOLE_INTERVAL, self._print_summary)

    # --- 1s summaries ---
    def _flush_summaries_all(self):
        for rs in self._robots.values():
            self._flush_summaries(rs)

    def _flush_summaries(self, rs: _RobotState):
        # cmd_vel summary
        buf = list(rs.cmd_vel_buf)
        rs.cmd_vel_buf.clear()

        if buf:
            lin_xs = [v[0] for v in buf]
            ang_zs = [v[1] for v in buf]
            mean_lin = sum(lin_xs) / len(lin_xs)
            mean_ang = sum(ang_zs) / len(ang_zs)
            max_abs_ang = max(abs(z) for z in ang_zs)
            is_moving = max_abs_ang > 0.005
        else:
            mean_lin = 0.0
            mean_ang = 0.0
            max_abs_ang = 0.0
            is_moving = False

        state = self._infer_state(rs, max_abs_ang, is_moving)
        rs.inferred_state = state

        p = (
            Point("motion")
            .field("mean_linear_x", float(mean_lin))
            .field("mean_angular_z", float(mean_ang))
            .field("max_abs_angular_z", float(max_abs_ang))
            .field("msg_count", len(buf))
            .field("is_moving", is_moving)
            .field("inferred_state", state)
        )
        self._write(rs, p)

        # odom snapshot
        odom = rs.odom_latest
        rs.odom_latest = None

        if odom is not None:
            p = (
                Point("odometry")
                .field("pos_x", float(odom.pose.pose.position.x))
                .field("pos_y", float(odom.pose.pose.position.y))
                .field("linear_vel", float(odom.twist.twist.linear.x))
                .field("angular_vel", float(odom.twist.twist.angular.z))
            )
            self._write(rs, p)

        # IMU summary (tuples: ax, ay, az, gz, qw, qx, qy, qz)
        imu_buf = list(rs.imu_buf)
        rs.imu_buf.clear()

        if imu_buf:
            n = len(imu_buf)
            ax = sum(s[0] for s in imu_buf) / n
            ay = sum(s[1] for s in imu_buf) / n
            az = sum(s[2] for s in imu_buf) / n
            gz = sum(s[3] for s in imu_buf) / n
            # Yaw from last quaternion
            last = imu_buf[-1]
            siny = 2.0 * (last[4] * last[7] + last[5] * last[6])
            cosy = 1.0 - 2.0 * (last[6] ** 2 + last[7] ** 2)
            yaw = math.atan2(siny, cosy)

            p = (
                Point("imu")
                .field("accel_x", float(ax))
                .field("accel_y", float(ay))
                .field("accel_z", float(az))
                .field("gyro_z", float(gz))
                .field("orientation_yaw", float(yaw))
            )
            self._write(rs, p)

    def _infer_state(self, rs: _RobotState, max_abs_ang: float, is_moving: bool) -> str:
        if not is_moving:
            if rs.is_docked:
                return "IDLE"
            cmd_stat = rs.stats.get("cmd_vel_unstamped")
            if cmd_stat and cmd_stat.gap() < 2.0:
                return "PAUSED"
            return "IDLE"

        if max_abs_ang > 0.06:
            return "ACTIVE_ROTATION"
        elif max_abs_ang > 0.005:
            return "ACTIVE_OSCILLATION"
        return "ACTIVE_ROTATION"

    # --- Network check (non-blocking) ---
    def _check_network_all(self):
        """Spawns a ping thread per robot — returns immediately."""
        for rs in self._robots.values():
            threading.Thread(
                target=self._ping_worker, args=(rs,), daemon=True
            ).start()

    def _ping_worker(self, rs: _RobotState):
        """Runs in a daemon thread — does NOT block the ROS2 executor."""
        ping_ms = self._ping_robot(rs.config.ip)
        rs.ping_ms = ping_ms
        wifi_dbm = self._get_wifi_signal()
        rs.wifi_dbm = wifi_dbm

        p = Point("network")
        if ping_ms is not None:
            p = p.field("ping_ms", float(ping_ms))
        if wifi_dbm is not None:
            p = p.field("wifi_signal_dbm", float(wifi_dbm))

        for name, stat in rs.stats.items():
            p = p.field(f"rate_{name}", float(round(stat.rate(), 2)))

        self._write(rs, p)

    def _ping_robot(self, ip: str) -> float | None:
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "2", ip],
                capture_output=True,
                text=True,
                timeout=3,
            )
            if result.returncode == 0:
                for line in result.stdout.split("\n"):
                    if "time=" in line:
                        ms = float(line.split("time=")[1].split()[0])
                        return ms
        except Exception:
            pass
        return None

    def _get_wifi_signal(self) -> float | None:
        try:
            result = subprocess.run(
                ["iwconfig"],
                capture_output=True,
                text=True,
                timeout=2,
            )
            for line in result.stdout.split("\n"):
                if "Signal level" in line:
                    part = line.split("Signal level=")[1].split()[0]
                    return float(part.replace("dBm", ""))
        except Exception:
            pass
        return None

    # --- SSH system poll (non-blocking) ---
    def _poll_system_all(self):
        """Spawns an SSH thread per robot — returns immediately."""
        for rs in self._robots.values():
            threading.Thread(
                target=self._ssh_worker, args=(rs,), daemon=True
            ).start()

    def _ssh_worker(self, rs: _RobotState):
        """Runs in a daemon thread — does NOT block the ROS2 executor."""
        cfg = rs.config
        try:
            cmd = (
                "cat /sys/class/thermal/thermal_zone0/temp && "
                "free -m | grep Mem && "
                "cat /proc/loadavg"
            )
            env = os.environ.copy()
            env["SSHPASS"] = cfg.ssh_password
            result = subprocess.run(
                [
                    "sshpass", "-e",
                    "ssh",
                    "-o", "PubkeyAuthentication=no",
                    "-o", "StrictHostKeyChecking=no",
                    "-o", "ConnectTimeout=5",
                    f"{cfg.ssh_user}@{cfg.ip}",
                    cmd,
                ],
                capture_output=True,
                text=True,
                timeout=10,
                env=env,
            )
            if result.returncode != 0:
                self.get_logger().warn(
                    f"SSH poll failed ({cfg.name}): {result.stderr.strip()[:100]}",
                    throttle_duration_sec=60.0,
                )
                return

            lines = result.stdout.strip().split("\n")
            if len(lines) < 3:
                return

            cpu_temp = float(lines[0]) / 1000.0
            mem_parts = lines[1].split()
            ram_total = float(mem_parts[1])
            ram_used = float(mem_parts[2])
            load_parts = lines[2].split()
            load_1m = float(load_parts[0])
            load_5m = float(load_parts[1])

            p = (
                Point("system")
                .field("cpu_temp_c", cpu_temp)
                .field("ram_used_mb", ram_used)
                .field("ram_total_mb", ram_total)
                .field("load_avg_1m", load_1m)
                .field("load_avg_5m", load_5m)
            )
            self._write(rs, p)

        except Exception as e:
            self.get_logger().warn(
                f"SSH poll error ({cfg.name}): {e}",
                throttle_duration_sec=30.0,
            )

    # --- Console summary ---
    def _print_summary(self):
        for rs in self._robots.values():
            parts = [f"[{rs.config.name}]"]
            if rs.battery_pct is not None:
                parts.append(f"BAT:{rs.battery_pct:.1f}%")
            if rs.battery_voltage is not None:
                parts.append(f"{rs.battery_voltage:.2f}V")
            if rs.battery_current is not None:
                parts.append(f"{rs.battery_current:.2f}A")
            parts.append(f"STATE:{rs.inferred_state}")
            if rs.ping_ms is not None:
                parts.append(f"PING:{rs.ping_ms:.0f}ms")
            else:
                parts.append("PING:--")
            if rs.wifi_dbm is not None:
                parts.append(f"WiFi:{rs.wifi_dbm:.0f}dBm")

            active = sum(1 for s in rs.stats.values() if s.gap() < 10.0)
            total = len(rs.stats)
            parts.append(f"TOPICS:{active}/{total}")

            now = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(f"[{now}] {' | '.join(parts)}")

    # -------------------------------------------------------------------
    # Shutdown
    # -------------------------------------------------------------------
    def shutdown(self):
        self.get_logger().info("Flushing InfluxDB buffer...")
        try:
            self._write_api.close()
        except Exception:
            pass
        try:
            self._influx.close()
        except Exception:
            pass
        self.get_logger().info("Robot Monitor shutdown complete")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def run(robots: list[RobotConfig]):
    """Spin a RobotMonitor node with the given robot configs."""
    rclpy.init()
    node = RobotMonitor(robots)
    shutting_down = False

    def _shutdown(sig=None, frame=None):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        node.get_logger().info("Signal received, shutting down...")
        node.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if not shutting_down:
            node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
