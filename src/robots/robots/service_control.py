#!/usr/bin/env python3
"""Web control panel for phase1 systemd service.

Standalone HTTP server (stdlib only) that lets staff enable/disable
the phase1 service from any phone on the TinyDancer WiFi network.

Endpoints:
    GET  /        — HTML control page
    GET  /status  — JSON { "active": bool, "enabled": bool }
    POST /start   — systemctl start phase1
    POST /stop    — systemctl stop phase1
"""

import json
import socket
import subprocess
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

PORT = 8090
SERVICE = "phase1"

# Track background start/stop operations
_pending_action = None  # "starting" or "stopping"
_action_lock = threading.Lock()


def get_robot_name():
    """Detect robot name from phase1 service file (e.g. robots:=Moon)."""
    try:
        with open("/etc/systemd/system/phase1.service") as f:
            for line in f:
                if "robots:=" in line:
                    # Extract name from e.g. "robots:=Moon" or "robots:=Basin"
                    part = line.split("robots:=")[1]
                    return part.strip().strip("'\"")
    except Exception:
        pass
    return socket.gethostname()


def service_status():
    """Return status string: 'active', 'activating', 'deactivating', 'inactive'."""
    result = subprocess.run(
        ["systemctl", "is-active", SERVICE],
        capture_output=True, text=True, timeout=10,
    )
    state = result.stdout.strip()  # "active", "inactive", "activating", etc.
    return state


def _do_start():
    global _pending_action
    try:
        subprocess.run(
            ["systemctl", "start", SERVICE],
            capture_output=True, text=True, timeout=120,
        )
    finally:
        with _action_lock:
            _pending_action = None


def _do_stop():
    global _pending_action
    try:
        subprocess.run(
            ["systemctl", "stop", SERVICE],
            capture_output=True, text=True, timeout=60,
        )
    finally:
        with _action_lock:
            _pending_action = None


def service_start():
    global _pending_action
    with _action_lock:
        if _pending_action:
            return False
        _pending_action = "starting"
    threading.Thread(target=_do_start, daemon=True).start()
    return True


def service_stop():
    global _pending_action
    with _action_lock:
        if _pending_action:
            return False
        _pending_action = "stopping"
    threading.Thread(target=_do_stop, daemon=True).start()
    return True


# Uses the same CSS variables and visual style as the existing dashboard
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{robot_name} — Robot Control</title>
<style>
:root {{
    --bg: #e8dff0;
    --card: #f3eef8;
    --text: #2d1b4e;
    --muted: #6b5a82;
    --border: rgba(80, 40, 120, 0.15);
    --accent: #7c3aed;
    --good: #45d483;
    --warn: #ffb020;
    --bad: #ef4444;
    --shadow: rgba(80, 40, 120, 0.12);
    --btn-bg: #c4b5d4;
    --btn-active: #4c1d95;
}}
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{
    font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif;
    background: radial-gradient(1200px 800px at 15% 10%, rgba(124,58,237,0.15), transparent 60%), var(--bg);
    color: var(--text);
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 2rem 1rem;
}}
h1 {{
    font-size: 1.75rem;
    margin-bottom: 0.25rem;
}}
.subtitle {{
    color: var(--muted);
    margin-bottom: 2rem;
    font-size: 0.9rem;
}}
.card {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 16px;
    padding: 2rem;
    box-shadow: 0 12px 26px var(--shadow);
    width: 100%;
    max-width: 400px;
    text-align: center;
}}
.status-label {{
    font-size: 0.85rem;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.1em;
    margin-bottom: 0.5rem;
}}
.status-indicator {{
    font-size: 1.5rem;
    font-weight: 700;
    margin-bottom: 2rem;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.5rem;
}}
.dot {{
    width: 12px;
    height: 12px;
    border-radius: 999px;
    display: inline-block;
    background: var(--bad);
    box-shadow: 0 0 0 3px rgba(239, 68, 68, 0.18);
}}
.dot.running {{
    background: var(--good);
    box-shadow: 0 0 0 3px rgba(69, 212, 131, 0.18);
}}
.dot.starting {{
    background: var(--warn);
    box-shadow: 0 0 0 3px rgba(255, 176, 32, 0.18);
    animation: pulse 1s infinite;
}}
@keyframes pulse {{
    0%, 100% {{ opacity: 1; }}
    50% {{ opacity: 0.4; }}
}}
.buttons {{
    display: flex;
    flex-direction: column;
    gap: 1rem;
}}
button {{
    font-family: inherit;
    font-size: 1.25rem;
    font-weight: 600;
    padding: 1rem 2rem;
    border: 1px solid var(--border);
    border-radius: 12px;
    cursor: pointer;
    color: var(--text);
    transition: border-color 0.2s, transform 0.1s;
    min-height: 60px;
}}
button:active {{ transform: scale(0.97); }}
.btn-enable {{
    background: var(--btn-active);
    border-color: var(--btn-active);
    color: #fff;
}}
.btn-enable:hover:enabled {{
    background: #5b21b6;
    border-color: #5b21b6;
}}
.btn-disable {{
    background: var(--btn-active);
    border-color: var(--btn-active);
    color: #fff;
}}
.btn-disable:hover:enabled {{
    background: #5b21b6;
    border-color: #5b21b6;
}}
button:disabled {{
    background: var(--btn-bg);
    border-color: var(--border);
    color: var(--muted);
    opacity: 0.6;
    cursor: not-allowed;
    transform: none;
}}
.error {{
    color: var(--bad);
    margin-top: 1rem;
    font-size: 0.9rem;
    min-height: 1.2em;
}}

/* Confirmation overlay */
.overlay {{
    display: none;
    position: fixed;
    inset: 0;
    background: rgba(0,0,0,0.7);
    z-index: 10;
    align-items: center;
    justify-content: center;
}}
.overlay.visible {{ display: flex; }}
.dialog {{
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 16px;
    padding: 2rem;
    box-shadow: 0 12px 26px var(--shadow);
    width: 90%;
    max-width: 360px;
    text-align: center;
}}
.dialog p {{
    margin-bottom: 1.5rem;
    font-size: 1.1rem;
}}
.dialog-buttons {{
    display: flex;
    gap: 1rem;
}}
.dialog-buttons button {{
    flex: 1;
    font-size: 1rem;
    padding: 0.75rem;
    min-height: 48px;
}}
.btn-cancel {{
    background: var(--btn-bg);
    border-color: var(--border);
    color: var(--text);
}}
.btn-cancel:hover {{
    background: #b8a8cc;
}}
</style>
</head>
<body>

<h1>{robot_name}</h1>
<p class="subtitle">Phase 1 Service Control</p>

<div class="card">
    <div class="status-label">Service Status</div>
    <div class="status-indicator">
        <span class="dot" id="dot"></span>
        <span id="status-text">Checking...</span>
    </div>
    <div class="buttons">
        <button class="btn-enable" id="btn-enable" onclick="confirmAction('start')">Enable</button>
        <button class="btn-disable" id="btn-disable" onclick="confirmAction('stop')">Disable</button>
    </div>
    <div class="error" id="error"></div>
</div>

<!-- Confirmation dialog -->
<div class="overlay" id="overlay">
    <div class="dialog">
        <p id="confirm-msg"></p>
        <div class="dialog-buttons">
            <button class="btn-cancel" onclick="hideDialog()">Cancel</button>
            <button id="confirm-btn" onclick="doAction()">Confirm</button>
        </div>
    </div>
</div>

<script>
let pendingAction = null;

function confirmAction(action) {{
    pendingAction = action;
    const verb = action === 'start' ? 'enable' : 'disable';
    document.getElementById('confirm-msg').textContent =
        'Are you sure you want to ' + verb + ' the phase1 service?';
    const btn = document.getElementById('confirm-btn');
    btn.textContent = verb.charAt(0).toUpperCase() + verb.slice(1);
    btn.className = action === 'start' ? 'btn-enable' : 'btn-disable';
    document.getElementById('overlay').classList.add('visible');
}}

function hideDialog() {{
    document.getElementById('overlay').classList.remove('visible');
    pendingAction = null;
}}

async function doAction() {{
    hideDialog();
    const action = pendingAction;
    if (!action) return;
    document.getElementById('error').textContent = '';
    try {{
        const resp = await fetch('/' + action, {{ method: 'POST' }});
        if (!resp.ok) {{
            const data = await resp.json();
            document.getElementById('error').textContent = data.error || 'Request failed';
        }}
        await refreshStatus();
    }} catch (e) {{
        document.getElementById('error').textContent = 'Connection error';
    }}
}}

async function refreshStatus() {{
    try {{
        const resp = await fetch('/status');
        const data = await resp.json();
        const dot = document.getElementById('dot');
        const text = document.getElementById('status-text');
        const btnEn = document.getElementById('btn-enable');
        const btnDis = document.getElementById('btn-disable');

        if (data.state === 'running') {{
            dot.className = 'dot running';
            text.textContent = 'Running';
            btnEn.disabled = true;
            btnDis.disabled = false;
        }} else if (data.state === 'starting') {{
            dot.className = 'dot starting';
            text.textContent = 'Starting...';
            btnEn.disabled = true;
            btnDis.disabled = true;
        }} else if (data.state === 'stopping') {{
            dot.className = 'dot';
            text.textContent = 'Stopping...';
            btnEn.disabled = true;
            btnDis.disabled = true;
        }} else {{
            dot.className = 'dot';
            text.textContent = 'Stopped';
            btnEn.disabled = false;
            btnDis.disabled = true;
        }}
        document.getElementById('error').textContent = '';
    }} catch (e) {{
        document.getElementById('error').textContent = 'Connection lost';
    }}
}}

// Initial load + auto-refresh every 3 seconds
refreshStatus();
setInterval(refreshStatus, 3000);
</script>
</body>
</html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            html = HTML_PAGE.format(robot_name=get_robot_name())
            self._respond(200, "text/html", html.encode())
        elif self.path == "/status":
            state = service_status()
            with _action_lock:
                pending = _pending_action
            # Map to a simple status the UI understands
            if pending == "starting" or state == "activating":
                ui_state = "starting"
            elif pending == "stopping" or state == "deactivating":
                ui_state = "stopping"
            elif state == "active":
                ui_state = "running"
            else:
                ui_state = "stopped"
            self._respond(
                200,
                "application/json",
                json.dumps({"state": ui_state}).encode(),
            )
        else:
            self._respond(404, "text/plain", b"Not Found")

    def do_POST(self):
        if self.path == "/start":
            ok = service_start()
            self._respond(
                200, "application/json",
                json.dumps({"ok": ok}).encode(),
            )
        elif self.path == "/stop":
            ok = service_stop()
            self._respond(
                200, "application/json",
                json.dumps({"ok": ok}).encode(),
            )
        else:
            self._respond(404, "text/plain", b"Not Found")

    def _respond(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        print(fmt % args)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def main():
    server = ThreadedHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Robot web control listening on port {PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    server.server_close()


if __name__ == "__main__":
    main()
