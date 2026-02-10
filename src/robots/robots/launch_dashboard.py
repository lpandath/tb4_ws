#!/usr/bin/env python3
"""
Phase 1 web dashboard: Table Performer UI + launch buttons and logs.
Run on laptop or Raspberry Pi; open http://<this_machine_ip>:8081
(Port 8080 is used by Create 3, so the dashboard uses 8081 by default.)

Usage (after sourcing your workspace):
  ros2 run robots launch_dashboard

Then open in browser: http://localhost:8081  or  http://<IP>:8081
Serves the dashboard (index, diagram, style) and /run, /stop, /status, /launches API.
"""

import json
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional, Tuple
from urllib.parse import parse_qs, urlparse
import tempfile

# Default port (8080 is Create 3); override with LAUNCH_DASHBOARD_PORT env
PORT = int(os.environ.get("LAUNCH_DASHBOARD_PORT", "8081"))

# Directory for per-launch logs (can be overridden with env var)
LOG_DIR = os.environ.get("LAUNCH_DASHBOARD_LOG_DIR", os.path.join(tempfile.gettempdir(), "robots_dashboard_logs"))
os.makedirs(LOG_DIR, exist_ok=True)

# Keep open file objects for launched processes so logs continue to be written
_log_files: dict = {}

# Launch names shown in UI -> (launch_file, description)
LAUNCHES = {
    "phase1": ("phase1.launch.py", "Phase 1 (clock swing)"),
    "moon_bringup": ("moon_bringup.launch.py", "Moon bringup: scan → relay → localisation (no nav)"),
    "moon_navigation": ("moon_navigation.launch.py", "Moon navigation"),
    "basin_scan": ("basin_scan.launch.py", "Basin scan + odom TF"),
    "basin_relay": ("basin_relay.launch.py", "Basin TF relay"),
    "basin_localization": ("basin_localization.launch.py", "Basin localization"),
    "basin_navigation": ("basin_navigation.launch.py", "Basin navigation"),
    "phase2_circle": ("phase2_circle.launch.py", "Phase 2 circle"),
}

# Scan and relay run in background (multiple at once) so movements work.
# Moon scan/relay are started via moon_bringup, so only Basin has standalone background entries.
BACKGROUND_LAUNCHES = {"basin_scan", "basin_relay"}

_foreground_process = None
_foreground_name: Optional[str] = None
_background_processes: dict = {}  # launch_key -> Popen


def _dashboard_dir() -> Optional[str]:
    """Return path to dashboard static files (install share or source tree)."""
    # 1) ament index (install share)
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory("robots")
        path = os.path.join(pkg_share, "dashboard")
        if os.path.isdir(path):
            return path
    except Exception:
        pass
    # 2) AMENT_PREFIX_PATH (e.g. after source install/setup.bash)
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        path = os.path.join(prefix, "share", "robots", "dashboard")
        if os.path.isdir(path):
            return path
    # 3) Source tree: robots/robots/launch_dashboard.py -> robots/dashboard
    this_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(this_dir, "..", "dashboard"))
    if os.path.isdir(path):
        return path
    return None


def run_launch(launch_key: str) -> Tuple[bool, str]:
    """Start ros2 launch. Background (scan/relay) can run alongside foreground. Returns (success, message)."""
    global _foreground_process, _foreground_name, _background_processes
    if launch_key not in LAUNCHES:
        return False, f"Unknown launch: {launch_key}"
    launch_file, _ = LAUNCHES[launch_key]

    if launch_key in BACKGROUND_LAUNCHES:
        proc = _background_processes.get(launch_key)
        if proc is not None and proc.poll() is None:
            return False, f"Already running in background: {launch_key}"
        try:
            # open (or create) a log file for this launch and stream both stdout/stderr
            log_path = os.path.join(LOG_DIR, f"dashboard_{launch_key}.log")
            logf = open(log_path, "ab")
            _log_files[launch_key] = logf
            _background_processes[launch_key] = subprocess.Popen(
                ["ros2", "launch", "robots", launch_file],
                env=os.environ,
                stdout=logf,
                stderr=subprocess.STDOUT,
            )
            return True, f"Started (background): {launch_key} (logs: {log_path})"
        except FileNotFoundError:
            return False, "ros2 not in PATH. Source your workspace: source install/setup.bash"
        except Exception as e:
            return False, str(e)

    # Foreground: only one at a time
    if _foreground_process is not None and _foreground_process.poll() is None:
        return False, f"Foreground already running: {_foreground_name}. Stop it first."
    try:
        # foreground launch also writes to its own log file
        log_path = os.path.join(LOG_DIR, f"dashboard_{launch_key}.log")
        logf = open(log_path, "ab")
        _log_files[launch_key] = logf
        _foreground_process = subprocess.Popen(
            ["ros2", "launch", "robots", launch_file],
            env=os.environ,
            stdout=logf,
            stderr=subprocess.STDOUT,
        )
        _foreground_name = launch_key
        return True, f"Started: {launch_key} (logs: {log_path})"
    except FileNotFoundError:
        return False, "ros2 not in PATH. Source your workspace: source install/setup.bash"
    except Exception as e:
        return False, str(e)


def stop_launch() -> Tuple[bool, str]:
    """Stop the foreground launch only. Background (scan/relay) keeps running."""
    global _foreground_process, _foreground_name
    if _foreground_process is None or _foreground_process.poll() is not None:
        _foreground_process = None
        _foreground_name = None
        return True, "No foreground launch running."
    try:
        _foreground_process.terminate()
        _foreground_process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        _foreground_process.kill()
    except Exception:
        pass
    name = _foreground_name
    _foreground_process = None
    _foreground_name = None
    # close any open foreground log file
    lf = _log_files.pop(name, None)
    try:
        if lf:
            lf.flush()
            lf.close()
    except Exception:
        pass
    return True, f"Stopped: {name}"


def get_status() -> str:
    """Foreground status; background runs are not blocking."""
    alive = []
    if _foreground_process is not None and _foreground_process.poll() is None:
        alive.append(f"Foreground: {_foreground_name}")
    bg = [k for k, p in _background_processes.items() if p.poll() is None]
    if bg:
        alive.append("Background: " + ", ".join(bg))
    # Prune dead background processes
    for k in list(_background_processes):
        if _background_processes[k].poll() is not None:
            del _background_processes[k]
    return "Idle" if not alive else " | ".join(alive)


# Content types for static files
STATIC_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
}


# CORS: allow the web UI (on laptop) to call this dashboard when it runs on a robot (by IP)
CORS_HEADERS = {"Access-Control-Allow-Origin": "*"}


class DashboardHandler(BaseHTTPRequestHandler):
    def _send_cors(self):
        for k, v in CORS_HEADERS.items():
            self.send_header(k, v)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"

        if path == "/status":
            self.send_response(200)
            self.send_header("Content-type", "text/plain")
            self._send_cors()
            self.end_headers()
            self.wfile.write(get_status().encode("utf-8"))
            return

        if path == "/launches":
            self.send_response(200)
            self.send_header("Content-type", "application/json; charset=utf-8")
            self._send_cors()
            self.end_headers()
            payload = [
                {"key": k, "description": desc}
                for k, (_, desc) in LAUNCHES.items()
            ]
            self.wfile.write(json.dumps(payload).encode("utf-8"))
            return

        if path == "/logs":
            qs = parse_qs(parsed.query)
            launch = (qs.get("launch") or [None])[0]
            try:
                lines = int((qs.get("lines") or [200])[0])
            except Exception:
                lines = 200
            if not launch:
                self.send_response(400)
                self.send_header("Content-type", "text/plain; charset=utf-8")
                self._send_cors()
                self.end_headers()
                self.wfile.write(b"Missing launch parameter")
                return
            log_path = os.path.join(LOG_DIR, f"dashboard_{launch}.log")
            if not os.path.isfile(log_path):
                self.send_response(404)
                self.send_header("Content-type", "text/plain; charset=utf-8")
                self._send_cors()
                self.end_headers()
                self.wfile.write(f"No log for launch: {launch}".encode("utf-8"))
                return
            # Read last `lines` lines (simple approach: read whole file)
            try:
                with open(log_path, "rb") as f:
                    data = f.read().decode("utf-8", errors="replace")
                tail = "\n".join(data.splitlines()[-lines:])
                self.send_response(200)
                self.send_header("Content-type", "text/plain; charset=utf-8")
                self._send_cors()
                self.end_headers()
                self.wfile.write(tail.encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-type", "text/plain; charset=utf-8")
                self._send_cors()
                self.end_headers()
                self.wfile.write(str(e).encode("utf-8"))
            return

        dashboard = _dashboard_dir()
        if dashboard:
            file_map = {
                "/": "index.html",
                "/index.html": "index.html",
                "/diagram.html": "diagram.html",
                "/style.css": "style.css",
            }
            file_name = file_map.get(path)
            if file_name:
                file_path = os.path.join(dashboard, file_name)
                if os.path.isfile(file_path):
                    try:
                        with open(file_path, "rb") as f:
                            data = f.read()
                    except OSError:
                        self._send_error("Dashboard file not readable.")
                        return
                    _, ext = os.path.splitext(file_name)
                    ct = STATIC_TYPES.get(ext, "application/octet-stream")
                    self.send_response(200)
                    self.send_header("Content-type", ct)
                    self._send_cors()
                    self.end_headers()
                    self.wfile.write(data)
                    return

        if path == "/" or path == "/index.html":
            self._send_error("Dashboard not found. Build and install: colcon build --packages-select robots")
            return

        self.send_response(404)
        self._send_cors()
        self.end_headers()

    def _send_error(self, message: str):
        self.send_response(503)
        self.send_header("Content-type", "text/plain; charset=utf-8")
        self._send_cors()
        self.end_headers()
        self.wfile.write(message.encode("utf-8"))

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/run":
            qs = parse_qs(parsed.query)
            launch = (qs.get("launch") or [None])[0]
            ok, msg = run_launch(launch) if launch else (False, "Missing launch parameter")
            self.send_response(200)
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self._send_cors()
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return
        if parsed.path == "/stop":
            ok, msg = stop_launch()
            self.send_response(200)
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self._send_cors()
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return
        self.send_response(404)
        self._send_cors()
        self.end_headers()

    def log_message(self, format, *args):
        sys.stderr.write("[dashboard] %s\n" % (format % args))


def main():
    server = HTTPServer(("0.0.0.0", PORT), DashboardHandler)
    print("Phase 1 dashboard: http://<this_ip>:%d  (Ctrl+C to stop)" % PORT)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
        if _foreground_process is not None and _foreground_process.poll() is None:
            _foreground_process.terminate()
        for p in _background_processes.values():
            if p.poll() is None:
                p.terminate()
        print("Stopped.")


if __name__ == "__main__":
    main()