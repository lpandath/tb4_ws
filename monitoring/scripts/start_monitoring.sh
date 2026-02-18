#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MON_DIR="$(dirname "$SCRIPT_DIR")"
cd "$MON_DIR"

# Load config
set -a
source .env
set +a

echo "=== Combined Production Monitor (Basin + Moon) ==="
echo "  Basin IP:  $BASIN_IP"
echo "  Moon IP:   $MOON_IP"
echo "  InfluxDB:  $INFLUXDB_URL"
echo "  Bucket:    $INFLUXDB_BUCKET"
echo ""

# Start Docker stack
echo "[1/3] Starting InfluxDB + Grafana..."
docker compose up -d

echo "[2/3] Waiting for InfluxDB health..."
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

echo "[3/3] Starting combined monitor (production)..."
echo ""
echo "  Grafana:  http://localhost:3000  (admin/admin)"
echo "  InfluxDB: http://localhost:8086  (admin/basinmonitor)"
echo ""
echo "  No rosbag — sole subscriber (half WiFi load per robot)"
echo "  Press Ctrl+C to stop monitoring"
echo "========================================="
echo ""

# Run monitor in foreground (Ctrl+C will stop it)
uv run --python python3.12 --with-requirements monitor/requirements.txt monitor/monitor.py
