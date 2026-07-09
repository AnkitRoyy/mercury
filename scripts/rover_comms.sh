#!/usr/bin/env zsh
cd "$(dirname "$0")/.."
source mercury.env

source ~/probes/mercury/install/setup.zsh   # ROS2 workspace

echo "[rover] starting logger..."
python3 communication/rover/rover_logger.py &
LOGGER_PID=$!

cleanup() {
    kill "${LOGGER_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2   # let first log files get created before bridge starts tailing

echo "[rover] starting bridge -> GCS at ${BASE_IP}..."
python3 communication/rover/rover_bridge.py --gcs-ip "${BASE_IP}" --log-dir "${LOG_DIR}"