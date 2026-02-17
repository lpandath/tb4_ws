#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MON_DIR="$(dirname "$SCRIPT_DIR")"
cd "$MON_DIR"

echo "=== Stopping Robot Monitors ==="

# Stop rosbag if running
if [ -f data/.rosbag_pid ]; then
    ROSBAG_PID=$(cat data/.rosbag_pid)
    if kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "  Stopping rosbag (PID $ROSBAG_PID)..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        sleep 2
    fi
    rm -f data/.rosbag_pid
fi

# Stop all monitors (basin, moon, combined, debug)
MONITOR_PIDS=$(pgrep -f "(basin_monitor|moon_monitor|monitor/monitor|basin_debug_monitor|moon_debug_monitor)\.py" 2>/dev/null || true)
if [ -n "$MONITOR_PIDS" ]; then
    echo "  Stopping monitor(s)..."
    echo "$MONITOR_PIDS" | xargs kill -INT 2>/dev/null || true
    sleep 2
fi

# Stop Docker
echo "  Stopping Docker containers..."
docker compose stop

echo ""
echo "  All stopped. Data persists in data/ directory."
echo "  Restart with: bash scripts/start_monitoring.sh"
echo "  To remove containers: docker compose down"
