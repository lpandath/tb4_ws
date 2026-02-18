#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MON_DIR="$(dirname "$SCRIPT_DIR")"
cd "$MON_DIR"

# Load config
set -a
source .env
set +a

echo "=== Basin Debug Monitor ==="
echo "  Basin IP:  $BASIN_IP"
echo "  InfluxDB:  $INFLUXDB_URL"
echo "  Bucket:    $INFLUXDB_BUCKET"
echo ""

# Start Docker stack
echo "[1/4] Starting InfluxDB + Grafana..."
docker compose up -d

echo "[2/4] Waiting for InfluxDB health..."
for i in $(seq 1 30); do
    if curl -sf http://localhost:8086/health > /dev/null 2>&1; then
        echo "  InfluxDB ready"
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "  WARNING: InfluxDB not responding after 30s, continuing anyway"
    fi
    sleep 1
done

# Start rosbag recording
ROSBAG_BASE="data/rosbags/basin_$(date +%Y%m%d_%H%M%S)"
mkdir -p data/rosbags

echo "[3/4] Starting rosbag recording → ${ROSBAG_BASE}/"
TOPICS=(
    --topics
    "/${BASIN_NAMESPACE#/}/battery_state"
    "/${BASIN_NAMESPACE#/}/cmd_vel_unstamped"
    "/${BASIN_NAMESPACE#/}/odom"
    "/${BASIN_NAMESPACE#/}/imu"
    "/${BASIN_NAMESPACE#/}/hazard_detection"
    "/${BASIN_NAMESPACE#/}/dock_status"
    "/${BASIN_NAMESPACE#/}/kidnap_status"
    "/${BASIN_NAMESPACE#/}/stop_status"
    "/${BASIN_NAMESPACE#/}/wheel_status"
    "/${BASIN_NAMESPACE#/}/ip"
    "/diagnostics"
    "/diagnostics_toplevel_state"
    "/rosout"
)

ros2 bag record \
    --storage mcap \
    --output "$ROSBAG_BASE" \
    "${TOPICS[@]}" &
ROSBAG_PID=$!
echo "  rosbag PID: $ROSBAG_PID"
ROSBAG_DIR="$ROSBAG_BASE"

# Save PIDs for stop script
echo "$ROSBAG_PID" > data/.rosbag_pid

echo "[4/4] Starting basin_debug_monitor..."
echo ""
echo "  Grafana:  http://localhost:3000  (admin/admin)"
echo "  InfluxDB: http://localhost:8086  (admin/basinmonitor)"
echo ""
echo "  Press Ctrl+C to stop monitoring"
echo "========================================="
echo ""

cleanup() {
    echo ""
    echo "Cleaning up..."
    if kill -0 "$ROSBAG_PID" 2>/dev/null; then
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait "$ROSBAG_PID" 2>/dev/null || true
        echo "  rosbag stopped"
    fi
    rm -f data/.rosbag_pid
    echo "  Docker containers still running (use stop_monitoring.sh to stop)"
    echo "  Rosbag saved: $ROSBAG_DIR"
}
trap cleanup EXIT

# Run debug monitor in foreground (Ctrl+C will stop it)
uv run --python python3.12 --with-requirements monitor/requirements.txt monitor/basin_debug_monitor.py
