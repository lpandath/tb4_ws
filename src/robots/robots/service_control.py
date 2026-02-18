#!/usr/bin/env python3
"""Web control panel for phase1 systemd services (Exhibition & Power Save).

Standalone HTTP server (stdlib only) that lets staff enable/disable
the phase1 or phase1-save service from any phone on the TinyDancer WiFi network.

Endpoints:
    GET  /            — HTML control page
    GET  /status      — JSON { "exhibition": state, "powersave": state }
    POST /start-exhibition  — start phase1, stop phase1-save
    POST /start-powersave   — start phase1-save, stop phase1
    POST /stop              — stop whichever is active
"""

import json
import socket
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

PORT = 8090
SVC_EXHIBITION = "phase1"
SVC_POWERSAVE = "phase1-save"


def get_robot_name():
    """Detect robot name from phase1 service file (e.g. robots:=Moon)."""
    try:
        with open("/etc/systemd/system/phase1.service") as f:
            for line in f:
                if "robots:=" in line:
                    part = line.split("robots:=")[1]
                    return part.strip().strip("'\"")
    except Exception:
        pass
    return socket.gethostname()


def _svc_state(service):
    """Return systemd state: 'active', 'activating', 'deactivating', 'inactive', etc."""
    result = subprocess.run(
        ["systemctl", "is-active", service],
        capture_output=True, text=True, timeout=10,
    )
    return result.stdout.strip()


def _ui_state(raw):
    if raw == "activating":
        return "starting"
    elif raw == "deactivating":
        return "stopping"
    elif raw == "active":
        return "running"
    return "stopped"


def _start(service):
    subprocess.Popen(["systemctl", "start", service],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _stop(service):
    subprocess.Popen(["systemctl", "stop", service],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


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
    --purple: #4c1d95;
    --brown: #6d4c2a;
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
.cards {{
    display: flex;
    flex-direction: column;
    gap: 1.25rem;
    width: 100%;
    max-width: 420px;
}}
.card {{
    background: var(--card);
    border: 2px solid var(--border);
    border-radius: 16px;
    padding: 1.5rem;
    box-shadow: 0 12px 26px var(--shadow);
    text-align: center;
    transition: border-color 0.3s;
}}
.card.active-exhibition {{
    border-color: var(--purple);
}}
.card.active-powersave {{
    border-color: var(--brown);
}}
.card-title {{
    font-size: 1.15rem;
    font-weight: 700;
    margin-bottom: 0.15rem;
}}
.card-desc {{
    font-size: 0.8rem;
    color: var(--muted);
    margin-bottom: 1rem;
}}
.status-row {{
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 0.5rem;
    margin-bottom: 1rem;
    font-size: 1.1rem;
    font-weight: 600;
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
button {{
    font-family: inherit;
    font-size: 1.1rem;
    font-weight: 600;
    padding: 0.85rem 1.5rem;
    border: 1px solid var(--border);
    border-radius: 12px;
    cursor: pointer;
    color: #fff;
    transition: border-color 0.2s, transform 0.1s;
    min-height: 52px;
    width: 100%;
}}
button:active {{ transform: scale(0.97); }}
button:disabled {{
    background: var(--btn-bg);
    border-color: var(--border);
    color: var(--muted);
    opacity: 0.6;
    cursor: not-allowed;
    transform: none;
}}
.btn-exhibition {{
    background: var(--purple);
    border-color: var(--purple);
}}
.btn-exhibition:hover:enabled {{
    background: #5b21b6;
}}
.btn-powersave {{
    background: var(--brown);
    border-color: var(--brown);
}}
.btn-powersave:hover:enabled {{
    background: #7a5530;
}}
.btn-disable {{
    background: var(--bad);
    border-color: var(--bad);
    margin-top: 0.5rem;
}}
.btn-disable:hover:enabled {{
    background: #dc2626;
}}
.error {{
    color: var(--bad);
    margin-top: 1rem;
    font-size: 0.9rem;
    min-height: 1.2em;
    text-align: center;
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

<div class="cards">
    <!-- Exhibition card -->
    <div class="card" id="card-exhibition">
        <div class="card-title">Exhibition Mode</div>
        <div class="card-desc">1.5 min active / 7 min rest</div>
        <div class="status-row">
            <span class="dot" id="dot-exhibition"></span>
            <span id="status-exhibition">Checking...</span>
        </div>
        <button class="btn-exhibition" id="btn-exhibition" onclick="confirmAction('start-exhibition')">
            Enable
        </button>
    </div>

    <!-- Power Save card -->
    <div class="card" id="card-powersave">
        <div class="card-title">Power Save Mode</div>
        <div class="card-desc">1 min active / 12 min rest</div>
        <div class="status-row">
            <span class="dot" id="dot-powersave"></span>
            <span id="status-powersave">Checking...</span>
        </div>
        <button class="btn-powersave" id="btn-powersave" onclick="confirmAction('start-powersave')">
            Enable Save
        </button>
    </div>

    <!-- Shared disable -->
    <button class="btn-disable" id="btn-disable" onclick="confirmAction('stop')">
        Disable
    </button>
</div>

<div class="error" id="error"></div>

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

const LABELS = {{
    'start-exhibition': {{ verb: 'enable Exhibition', btnClass: 'btn-exhibition' }},
    'start-powersave':  {{ verb: 'enable Save mode', btnClass: 'btn-powersave' }},
    'stop':             {{ verb: 'disable the active service', btnClass: 'btn-disable' }},
}};

function confirmAction(action) {{
    pendingAction = action;
    const info = LABELS[action];
    document.getElementById('confirm-msg').textContent =
        'Are you sure you want to ' + info.verb + '?';
    const btn = document.getElementById('confirm-btn');
    btn.textContent = 'Confirm';
    btn.className = info.btnClass;
    document.getElementById('overlay').classList.add('visible');
}}

function hideDialog() {{
    document.getElementById('overlay').classList.remove('visible');
    pendingAction = null;
}}

function disableAll() {{
    document.getElementById('btn-exhibition').disabled = true;
    document.getElementById('btn-powersave').disabled = true;
    document.getElementById('btn-disable').disabled = true;
}}

async function doAction() {{
    const action = pendingAction;
    hideDialog();
    if (!action) return;
    document.getElementById('error').textContent = '';
    disableAll();

    // Immediate visual feedback
    if (action === 'start-exhibition') {{
        document.getElementById('dot-exhibition').className = 'dot starting';
        document.getElementById('status-exhibition').textContent = 'Starting...';
        document.getElementById('dot-powersave').className = 'dot';
        document.getElementById('status-powersave').textContent = 'Stopping...';
    }} else if (action === 'start-powersave') {{
        document.getElementById('dot-powersave').className = 'dot starting';
        document.getElementById('status-powersave').textContent = 'Starting...';
        document.getElementById('dot-exhibition').className = 'dot';
        document.getElementById('status-exhibition').textContent = 'Stopping...';
    }} else {{
        document.getElementById('dot-exhibition').className = 'dot';
        document.getElementById('status-exhibition').textContent = 'Stopping...';
        document.getElementById('dot-powersave').className = 'dot';
        document.getElementById('status-powersave').textContent = 'Stopping...';
    }}

    try {{
        await fetch('/' + action, {{ method: 'POST' }});
    }} catch (e) {{
        document.getElementById('error').textContent = 'Connection error';
        // Re-enable all buttons so the user is never stuck
        document.getElementById('btn-exhibition').disabled = false;
        document.getElementById('btn-powersave').disabled = false;
        document.getElementById('btn-disable').disabled = false;
    }}
}}

async function refreshStatus() {{
    try {{
        const resp = await fetch('/status');
        const data = await resp.json();
        const exh = data.exhibition;
        const ps  = data.powersave;

        // Exhibition dot/text
        const dotE = document.getElementById('dot-exhibition');
        const txtE = document.getElementById('status-exhibition');
        const cardE = document.getElementById('card-exhibition');
        if (exh === 'running')       {{ dotE.className = 'dot running';  txtE.textContent = 'Running'; }}
        else if (exh === 'starting') {{ dotE.className = 'dot starting'; txtE.textContent = 'Starting...'; }}
        else if (exh === 'stopping') {{ dotE.className = 'dot';          txtE.textContent = 'Stopping...'; }}
        else                         {{ dotE.className = 'dot';          txtE.textContent = 'Stopped'; }}

        // Power Save dot/text
        const dotP = document.getElementById('dot-powersave');
        const txtP = document.getElementById('status-powersave');
        const cardP = document.getElementById('card-powersave');
        if (ps === 'running')       {{ dotP.className = 'dot running';  txtP.textContent = 'Running'; }}
        else if (ps === 'starting') {{ dotP.className = 'dot starting'; txtP.textContent = 'Starting...'; }}
        else if (ps === 'stopping') {{ dotP.className = 'dot';          txtP.textContent = 'Stopping...'; }}
        else                        {{ dotP.className = 'dot';          txtP.textContent = 'Stopped'; }}

        // Active card highlight
        cardE.className = exh === 'running' || exh === 'starting' ? 'card active-exhibition' : 'card';
        cardP.className = ps  === 'running' || ps  === 'starting' ? 'card active-powersave'  : 'card';

        // Button states — each button only cares about its own service
        const anyActive = (exh !== 'stopped' || ps !== 'stopped');

        document.getElementById('btn-exhibition').disabled = (exh !== 'stopped');
        document.getElementById('btn-powersave').disabled  = (ps  !== 'stopped');
        // Disable button is ALWAYS available when anything is non-stopped
        document.getElementById('btn-disable').disabled    = !anyActive;

        document.getElementById('error').textContent = '';
    }} catch (e) {{
        document.getElementById('error').textContent = 'Connection lost';
        // Re-enable all buttons so the user is never stuck
        document.getElementById('btn-exhibition').disabled = false;
        document.getElementById('btn-powersave').disabled = false;
        document.getElementById('btn-disable').disabled = false;
    }}
}}

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
            exh_state = _ui_state(_svc_state(SVC_EXHIBITION))
            ps_state = _ui_state(_svc_state(SVC_POWERSAVE))
            self._respond(
                200,
                "application/json",
                json.dumps({"exhibition": exh_state, "powersave": ps_state}).encode(),
            )
        else:
            self._respond(404, "text/plain", b"Not Found")

    def do_POST(self):
        if self.path == "/start-exhibition":
            _stop(SVC_POWERSAVE)
            _start(SVC_EXHIBITION)
            self._respond(200, "application/json", json.dumps({"ok": True}).encode())
        elif self.path == "/start-powersave":
            _stop(SVC_EXHIBITION)
            _start(SVC_POWERSAVE)
            self._respond(200, "application/json", json.dumps({"ok": True}).encode())
        elif self.path == "/stop":
            _stop(SVC_EXHIBITION)
            _stop(SVC_POWERSAVE)
            self._respond(200, "application/json", json.dumps({"ok": True}).encode())
        else:
            self._respond(404, "text/plain", b"Not Found")

    def _respond(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
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
