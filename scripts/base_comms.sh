#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source mercury.env

echo "[base] starting bridge (WS :9090, MJPEG :8080, tiles :8000)..."
python3 communication/base/base_bridge.py --rover-ip "${ROVER_IP}" --tile-dir "${TILE_DIR}" &
BRIDGE_PID=$!

cleanup() {
    kill "${BRIDGE_PID}" "${GUI_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 1
echo "[base] starting GUI dev server..."
(cd communication/base/gui && npm run dev) &
GUI_PID=$!

wait