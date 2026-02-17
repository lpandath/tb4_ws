#!/usr/bin/env python3
"""
Moon Battery Lifecycle Monitor (Debug)
=======================================
Full-capture debug monitor for Moon robot. Subscribes to all 14 topics
including scan, diagnostics, and rosout. Intended to run alongside rosbag.

Usage:
    python3 moon_debug_monitor.py
"""

import math
import os
import signal
import subprocess
import sys
import time
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

# Standard ROS2 messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import Log

# iRobot Create 3 messages
from irobot_create_msgs.msg import (
    DockStatus,
    HazardDetectionVector,
    KidnapStatus,
    StopStatus,
    WheelStatus,
)

from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS, WriteOptions

# ---------------------------------------------------------------------------
# Configuration from environment
# ---------------------------------------------------------------------------
MOON_IP = os.environ.get("MOON_IP", "192.168.1.227")
MOON_SSH_USER = os.environ.get("SSH_USER", "ubuntu")
MOON_SSH_PASSWORD = os.environ.get("SSH_PASSWORD", "turtlebot4")
MOON_NS = os.environ.get("MOON_NAMESPACE", "/Moon").strip("/")

INFLUXDB_URL = os.environ.get("INFLUXDB_URL", "http://localhost:8086")
INFLUXDB_TOKEN = os.environ.get("INFLUXDB_TOKEN", "basin-monitor-token-2026")
INFLUXDB_ORG = os.environ.get("INFLUXDB_ORG", "turtlebot")
INFLUXDB_BUCKET = os.environ.get("INFLUXDB_BUCKET", "exhibition")

CONSOLE_INTERVAL = float(os.environ.get("CONSOLE_INTERVAL", "30"))
SSH_POLL_INTERVAL = float(os.environ.get("SSH_POLL_INTERVAL", "30"))
NETWORK_CHECK_INTERVAL = float(os.environ.get("NETWORK_CHECK_INTERVAL", "5"))


# ---------------------------------------------------------------------------
# Topic rate tracker
# ---------------------------------------------------------------------------
class TopicStats:
    """Track message rates and gaps for a single topic."""

    def __init__(self, window_sec=60.0):
        self._times = deque()
        self._window = window_sec
        self._lock = threading.Lock()
        self.last_time = 0.0
        self.msg_count = 0

    def record(self):
        now = time.time()
        with self._lock:
            self._times.append(now)
            self.last_time = now
            self.msg_count += 1
            cutoff = now - self._window
            while self._times and self._times[0] < cutoff:
                self._times.popleft()

    def rate(self) -> float:
        """Messages per second over the sliding window."""
        with self._lock:
            if len(self._times) < 2:
                return 0.0
            span = self._times[-1] - self._times[0]
            if span <= 0:
                return 0.0
            return (len(self._times) - 1) / span

    def gap(self) -> float:
        """Seconds since last message."""
        if self.last_time == 0:
            return float("inf")
        return time.time() - self.last_time


# ---------------------------------------------------------------------------
# Main monitor node
# ---------------------------------------------------------------------------
class MoonMonitor(Node):

    def __init__(self):
        super().__init__("moon_monitor")
        self.get_logger().info(f"Moon Monitor starting — NS=/{MOON_NS}, IP={MOON_IP}")

        # InfluxDB writer with batching
        self._influx = InfluxDBClient(
            url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG
        )
        self._write_api = self._influx.write_api(
            write_options=WriteOptions(
                batch_size=1000,
                flush_interval=5_000,  # ms
                jitter_interval=0,
                retry_interval=5_000,
            )
        )

        # Per-topic stats
        self._stats: dict[str, TopicStats] = {}

        # Latest values for console summary
        self._battery_pct = None
        self._battery_voltage = None
        self._battery_current = None
        self._inferred_state = "UNKNOWN"
        self._ping_ms = None
        self._wifi_dbm = None
        self._is_docked = None

        # SSH via sshpass + subprocess
        self._ssh_cmd_prefix = [
            "sshpass", f"-p{MOON_SSH_PASSWORD}",
            "ssh",
            "-o", "PubkeyAuthentication=no",
            "-o", "StrictHostKeyChecking=no",
            "-o", "ConnectTimeout=5",
            f"{MOON_SSH_USER}@{MOON_IP}",
        ]

        # Accumulated cmd_vel samples for 1s summary
        self._cmd_vel_buf: list[tuple[float, float]] = []
        self._cmd_vel_lock = threading.Lock()

        # Accumulated odom samples
        self._odom_latest = None
        self._odom_lock = threading.Lock()

        # Accumulated IMU samples
        self._imu_buf: list[dict] = []
        self._imu_lock = threading.Lock()

        # Last dock/kidnap/stop values for on-change detection
        self._last_dock_status = None
        self._last_kidnap = None
        self._last_stop = None

        self._setup_subscriptions()
        self._setup_timers()

        self.get_logger().info("Subscriptions and timers ready")

    # -------------------------------------------------------------------
    # Subscriptions
    # -------------------------------------------------------------------
    def _setup_subscriptions(self):
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
        qos_rosout = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

        ns = MOON_NS

        # 1. Battery
        self._sub(f"/{ns}/battery_state", BatteryState, self._on_battery, qos_sensor)

        # 2. cmd_vel_unstamped (Twist) — high rate, buffer for 1s summary
        self._sub(f"/{ns}/cmd_vel_unstamped", Twist, self._on_cmd_vel, qos_reliable)

        # 3. Odometry — high rate, snapshot 1s
        self._sub(f"/{ns}/odom", Odometry, self._on_odom, qos_sensor)

        # 4. IMU — high rate, summary 1s
        self._sub(f"/{ns}/imu", Imu, self._on_imu, qos_sensor)

        # 5. LaserScan — metadata only every 2s
        self._scan_last_write = 0.0
        self._sub(f"/{ns}/scan", LaserScan, self._on_scan, qos_sensor)

        # 6. Hazard detection — every event
        self._sub(
            f"/{ns}/hazard_detection",
            HazardDetectionVector,
            self._on_hazard,
            qos_sensor,
        )

        # 7. Dock status — on change
        self._sub(f"/{ns}/dock_status", DockStatus, self._on_dock, qos_sensor)

        # 8. Kidnap status — on change
        self._sub(f"/{ns}/kidnap_status", KidnapStatus, self._on_kidnap, qos_sensor)

        # 9. Stop status — on change
        self._sub(f"/{ns}/stop_status", StopStatus, self._on_stop, qos_sensor)

        # 10. Wheel status — on change
        self._last_wheel_time = 0.0
        self._sub(f"/{ns}/wheel_status", WheelStatus, self._on_wheel, qos_sensor)

        # 11. IP address — on change
        self._last_ip = None
        self._sub(f"/{ns}/ip", String, self._on_ip, qos_reliable)

        # 12. Diagnostics
        self._sub("/diagnostics", DiagnosticArray, self._on_diagnostics, qos_reliable)

        # 13. Top-level diagnostics
        self._sub(
            "/diagnostics_toplevel_state",
            DiagnosticStatus,
            self._on_health,
            qos_reliable,
        )

        # 14. rosout (WARN+ only)
        self._sub("/rosout", Log, self._on_rosout, qos_rosout)

    def _sub(self, topic, msg_type, callback, qos):
        """Create subscription and register a TopicStats tracker."""
        name = topic.split("/")[-1]
        self._stats[name] = TopicStats()
        self.create_subscription(msg_type, topic, callback, qos)

    def _stat(self, topic_name):
        return self._stats.get(topic_name)

    # -------------------------------------------------------------------
    # InfluxDB write helper
    # -------------------------------------------------------------------
    def _write(self, point: Point):
        try:
            self._write_api.write(bucket=INFLUXDB_BUCKET, record=point)
        except Exception as e:
            self.get_logger().warn(f"InfluxDB write error: {e}", throttle_duration_sec=10.0)

    # -------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------
    def _on_battery(self, msg: BatteryState):
        self._stat("battery_state").record()
        pct = msg.percentage
        # Create 3 reports 0-1 range
        if pct <= 1.0:
            pct *= 100.0
        self._battery_pct = pct
        self._battery_voltage = msg.voltage
        self._battery_current = msg.current

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
        self._write(p)

    def _on_cmd_vel(self, msg: Twist):
        self._stat("cmd_vel_unstamped").record()
        with self._cmd_vel_lock:
            self._cmd_vel_buf.append((msg.linear.x, msg.angular.z))

    def _on_odom(self, msg: Odometry):
        self._stat("odom").record()
        with self._odom_lock:
            self._odom_latest = msg

    def _on_imu(self, msg: Imu):
        self._stat("imu").record()
        with self._imu_lock:
            self._imu_buf.append(
                {
                    "ax": msg.linear_acceleration.x,
                    "ay": msg.linear_acceleration.y,
                    "az": msg.linear_acceleration.z,
                    "gz": msg.angular_velocity.z,
                    "qw": msg.orientation.w,
                    "qx": msg.orientation.x,
                    "qy": msg.orientation.y,
                    "qz": msg.orientation.z,
                }
            )

    def _on_scan(self, msg: LaserScan):
        self._stat("scan").record()
        now = time.time()
        if now - self._scan_last_write < 2.0:
            return
        self._scan_last_write = now

        valid = [r for r in msg.ranges if 0.15 < r < 5.0 and not math.isinf(r)]
        p = (
            Point("lidar")
            .field("closest_range", float(min(valid)) if valid else 0.0)
            .field("mean_range", float(sum(valid) / len(valid)) if valid else 0.0)
            .field("valid_ray_count", len(valid))
            .field("total_ray_count", len(msg.ranges))
        )
        self._write(p)

    def _on_hazard(self, msg: HazardDetectionVector):
        self._stat("hazard_detection").record()
        for det in msg.detections:
            p = (
                Point("hazards")
                .field("hazard_type", int(det.type))
                .field("description", det.header.frame_id)
            )
            self._write(p)

    def _on_dock(self, msg: DockStatus):
        self._stat("dock_status").record()
        val = (msg.is_docked, msg.dock_visible)
        if val == self._last_dock_status:
            return
        self._last_dock_status = val
        self._is_docked = msg.is_docked

        p = (
            Point("dock")
            .field("is_docked", msg.is_docked)
            .field("dock_visible", msg.dock_visible)
        )
        self._write(p)

    def _on_kidnap(self, msg: KidnapStatus):
        self._stat("kidnap_status").record()
        val = msg.is_kidnapped
        if val == self._last_kidnap:
            return
        self._last_kidnap = val

        p = (
            Point("events")
            .tag("source", "kidnap")
            .field("value", val)
            .field("description", "kidnapped" if val else "restored")
        )
        self._write(p)

    def _on_stop(self, msg: StopStatus):
        self._stat("stop_status").record()
        val = msg.is_stopped
        if val == self._last_stop:
            return
        self._last_stop = val

        p = (
            Point("events")
            .tag("source", "stop")
            .field("value", val)
            .field("description", "stopped" if val else "released")
        )
        self._write(p)

    def _on_wheel(self, msg: WheelStatus):
        self._stat("wheel_status").record()
        now = time.time()
        # Throttle to 1Hz
        if now - self._last_wheel_time < 1.0:
            return
        self._last_wheel_time = now

        p = (
            Point("wheels")
            .field("current_ma_left", int(msg.current_ma_left))
            .field("current_ma_right", int(msg.current_ma_right))
            .field("pwm_left", int(msg.pwm_left))
            .field("pwm_right", int(msg.pwm_right))
        )
        self._write(p)

    def _on_ip(self, msg: String):
        self._stat("ip").record()
        val = msg.data
        if val == self._last_ip:
            return
        self._last_ip = val

        p = (
            Point("events")
            .tag("source", "ip")
            .field("value", True)
            .field("description", val)
        )
        self._write(p)

    def _on_diagnostics(self, msg: DiagnosticArray):
        self._stat("diagnostics").record()
        for status in msg.status:
            if status.level == DiagnosticStatus.OK:
                continue
            kv = "; ".join(f"{kv.key}={kv.value}" for kv in status.values[:10])
            p = (
                Point("diagnostics")
                .tag("name", status.name[:64])
                .field("level", int(status.level))
                .field("message", status.message[:256])
                .field("kv_json", kv[:512])
            )
            self._write(p)

    def _on_health(self, msg: DiagnosticStatus):
        self._stat("diagnostics_toplevel_state").record()
        p = (
            Point("health")
            .field("level", int(msg.level))
            .field("message", msg.message[:256])
        )
        self._write(p)

    def _on_rosout(self, msg: Log):
        self._stat("rosout").record()
        # Only WARN (4) and above
        if msg.level < Log.WARN:
            return
        p = (
            Point("logs")
            .tag("node_name", msg.name[:64])
            .field("level", int(msg.level))
            .field("message", msg.msg[:512])
        )
        self._write(p)

    # -------------------------------------------------------------------
    # Timers
    # -------------------------------------------------------------------
    def _setup_timers(self):
        # 1s: flush cmd_vel / odom / imu summaries + state inference
        self.create_timer(1.0, self._flush_summaries)

        # Network check
        self.create_timer(NETWORK_CHECK_INTERVAL, self._check_network)

        # SSH system poll
        self.create_timer(SSH_POLL_INTERVAL, self._poll_system)

        # Console summary
        self.create_timer(CONSOLE_INTERVAL, self._print_summary)

    def _flush_summaries(self):
        """Write 1s summaries for high-rate topics + infer state."""
        # --- cmd_vel summary ---
        with self._cmd_vel_lock:
            buf = list(self._cmd_vel_buf)
            self._cmd_vel_buf.clear()

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

        # State inference
        state = self._infer_state(mean_ang, max_abs_ang, is_moving)
        self._inferred_state = state

        p = (
            Point("motion")
            .field("mean_linear_x", float(mean_lin))
            .field("mean_angular_z", float(mean_ang))
            .field("max_abs_angular_z", float(max_abs_ang))
            .field("msg_count", len(buf))
            .field("is_moving", is_moving)
            .field("inferred_state", state)
        )
        self._write(p)

        # --- odom snapshot ---
        with self._odom_lock:
            odom = self._odom_latest
            self._odom_latest = None

        if odom is not None:
            p = (
                Point("odometry")
                .field("pos_x", float(odom.pose.pose.position.x))
                .field("pos_y", float(odom.pose.pose.position.y))
                .field("linear_vel", float(odom.twist.twist.linear.x))
                .field("angular_vel", float(odom.twist.twist.angular.z))
            )
            self._write(p)

        # --- IMU summary ---
        with self._imu_lock:
            imu_buf = list(self._imu_buf)
            self._imu_buf.clear()

        if imu_buf:
            n = len(imu_buf)
            ax = sum(s["ax"] for s in imu_buf) / n
            ay = sum(s["ay"] for s in imu_buf) / n
            az = sum(s["az"] for s in imu_buf) / n
            gz = sum(s["gz"] for s in imu_buf) / n
            # Yaw from last quaternion
            last = imu_buf[-1]
            siny = 2.0 * (last["qw"] * last["qz"] + last["qx"] * last["qy"])
            cosy = 1.0 - 2.0 * (last["qy"] ** 2 + last["qz"] ** 2)
            yaw = math.atan2(siny, cosy)

            p = (
                Point("imu")
                .field("accel_x", float(ax))
                .field("accel_y", float(ay))
                .field("accel_z", float(az))
                .field("gyro_z", float(gz))
                .field("orientation_yaw", float(yaw))
            )
            self._write(p)

    def _infer_state(self, mean_ang: float, max_abs_ang: float, is_moving: bool) -> str:
        """Infer robot state from cmd_vel patterns."""
        if not is_moving:
            if self._is_docked:
                return "IDLE"
            # Check if cmd_vel topic is active (controller running but paused)
            cmd_stat = self._stat("cmd_vel_unstamped")
            if cmd_stat and cmd_stat.gap() < 2.0:
                return "PAUSED"
            return "IDLE"

        # Detect rotation vs oscillation from angular velocity pattern
        if max_abs_ang > 0.06:
            return "ACTIVE_ROTATION"
        elif max_abs_ang > 0.005:
            return "ACTIVE_OSCILLATION"
        return "ACTIVE_ROTATION"

    def _check_network(self):
        """Ping Moon and collect WiFi signal, write topic rates."""
        ping_ms = self._ping_moon()
        self._ping_ms = ping_ms
        wifi_dbm = self._get_wifi_signal()
        self._wifi_dbm = wifi_dbm

        p = Point("network")
        if ping_ms is not None:
            p = p.field("ping_ms", float(ping_ms))
        if wifi_dbm is not None:
            p = p.field("wifi_signal_dbm", float(wifi_dbm))

        # Per-topic rates
        for name, stat in self._stats.items():
            p = p.field(f"rate_{name}", float(round(stat.rate(), 2)))

        self._write(p)

    def _ping_moon(self) -> float | None:
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "2", MOON_IP],
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
                    # Format: Signal level=-XX dBm
                    part = line.split("Signal level=")[1].split()[0]
                    return float(part.replace("dBm", ""))
        except Exception:
            pass
        return None

    def _poll_system(self):
        """SSH to Moon Pi for CPU temp, RAM, load average."""
        try:
            cmd = (
                "cat /sys/class/thermal/thermal_zone0/temp && "
                "free -m | grep Mem && "
                "cat /proc/loadavg"
            )
            result = subprocess.run(
                self._ssh_cmd_prefix + [cmd],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                self.get_logger().warn(
                    f"SSH poll failed: {result.stderr.strip()[:100]}",
                    throttle_duration_sec=60.0,
                )
                return

            lines = result.stdout.strip().split("\n")
            if len(lines) < 3:
                return

            # CPU temp (millidegrees -> Celsius)
            cpu_temp = float(lines[0]) / 1000.0

            # RAM: Mem: total used free shared buff/cache available
            mem_parts = lines[1].split()
            ram_total = float(mem_parts[1])
            ram_used = float(mem_parts[2])

            # Load average
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
            self._write(p)

        except Exception as e:
            self.get_logger().warn(f"SSH poll error: {e}", throttle_duration_sec=30.0)

    def _print_summary(self):
        """Print console status summary."""
        parts = []
        if self._battery_pct is not None:
            parts.append(f"BAT:{self._battery_pct:.1f}%")
        if self._battery_voltage is not None:
            parts.append(f"{self._battery_voltage:.2f}V")
        if self._battery_current is not None:
            parts.append(f"{self._battery_current:.2f}A")
        parts.append(f"STATE:{self._inferred_state}")
        if self._ping_ms is not None:
            parts.append(f"PING:{self._ping_ms:.0f}ms")
        else:
            parts.append("PING:--")
        if self._wifi_dbm is not None:
            parts.append(f"WiFi:{self._wifi_dbm:.0f}dBm")

        # Topic rates
        active = sum(1 for s in self._stats.values() if s.gap() < 10.0)
        total = len(self._stats)
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
        self.get_logger().info("Moon Monitor shutdown complete")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = MoonMonitor()
    shutting_down = False

    def _shutdown(sig=None, frame=None):
        nonlocal shutting_down
        if shutting_down:
            return
        shutting_down = True
        node.get_logger().info("Signal received, shutting down...")
        node.shutdown()
        rclpy.shutdown()

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


if __name__ == "__main__":
    main()
