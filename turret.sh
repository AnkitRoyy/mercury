#!/usr/bin/env bash
#
# launch_turret_vision.sh
#
# 1. colcon builds turret_vision (blocking, in this pane, before anything else).
# 2. Opens 4 new kitty splits, each sourcing ROS + workspace setup, running:
#      - this pane:  ros2 launch turret_vision turret_vision.launch.py
#      - split 1:    ros2 run turret_vision scanner  --ros-args --params-file ...
#      - split 2:    ros2 run turret_vision turret   --ros-args --params-file ...
#      - split 3:    ros2 run turret_vision trigger  --ros-args --params-file ...
#      - split 4:    waits a few seconds (let the above come up), then fires
#                     `ros2 topic pub --once /start std_msgs/msg/Bool "{data: true}"`
#
# Requirements:
#   - kitty terminal with `allow_remote_control yes` set in kitty.conf,
#     with kitty fully restarted after that setting was added
#   - Run from your ROS 2 workspace root, so install/setup.zsh and
#     src/turret_vision/config/vision_params.yaml resolve relative to cwd
set -euo pipefail

WS_DIR="$(pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.zsh"
WS_SETUP="${WS_DIR}/install/setup.zsh"
PARAMS_FILE="src/turret_vision/config/vision_params.yaml"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "Error: $ROS_SETUP not found." >&2
  exit 1
fi

if [[ ! -f "${WS_DIR}/${PARAMS_FILE}" ]]; then
  echo "Error: ${PARAMS_FILE} not found under ${WS_DIR}. Run this script from your workspace root." >&2
  exit 1
fi

if ! command -v kitty >/dev/null 2>&1; then
  echo "Error: kitty not found in PATH." >&2
  exit 1
fi

# ── colcon build first, blocking, before we touch install/setup.zsh ──────────
echo "Building turret_vision..."
colcon build --packages-select turret_vision
echo "Build complete."

if [[ ! -f "$WS_SETUP" ]]; then
  echo "Error: $WS_SETUP not found after build. Something went wrong." >&2
  exit 1
fi

# Figure out how to reach kitty's remote-control socket.
KITTY_SOCKET="${KITTY_LISTEN_ON:-unix:/tmp/kitty}"
KITTY_CMD=(kitty @ --to="$KITTY_SOCKET")

if ! "${KITTY_CMD[@]}" ls >/dev/null 2>&1; then
  echo "Error: could not reach kitty over remote control at '$KITTY_SOCKET'." >&2
  echo "Things to check:" >&2
  echo "  1. 'allow_remote_control yes' is set in kitty.conf AND kitty was fully" >&2
  echo "     restarted (quit all kitty windows, not just this tab) after adding it." >&2
  echo "  2. This script is being run in a normal shell, not one that's lost" >&2
  echo "     KITTY_LISTEN_ON (e.g. via sudo, or a stripped-env subshell)." >&2
  echo "  3. Try manually: kitty @ --to=\"$KITTY_SOCKET\" ls" >&2
  exit 1
fi

# ── Scanner node ──────────────────────────────────────────────────────────────
SCANNER_CMD=$(cat <<EOF
zsh -i -c '
source "${ROS_SETUP}"
source "${WS_SETUP}"
ros2 run turret_vision scanner --ros-args --params-file ${PARAMS_FILE}
echo
echo "[pane] process exited, dropping to shell"
exec zsh
'
EOF
)

# ── Turret node ───────────────────────────────────────────────────────────────
TURRET_CMD=$(cat <<EOF
zsh -i -c '
source "${ROS_SETUP}"
source "${WS_SETUP}"
ros2 run turret_vision turret --ros-args --params-file ${PARAMS_FILE}
echo
echo "[pane] process exited, dropping to shell"
exec zsh
'
EOF
)

# ── Trigger node ──────────────────────────────────────────────────────────────
TRIGGER_CMD=$(cat <<EOF
zsh -i -c '
source "${ROS_SETUP}"
source "${WS_SETUP}"
ros2 run turret_vision trigger --ros-args --params-file ${PARAMS_FILE}
echo
echo "[pane] process exited, dropping to shell"
exec zsh
'
EOF
)

# ── /start publisher (delayed so the other nodes are up first) ───────────────
START_PUB_CMD=$(cat <<EOF
zsh -i -c '
source "${ROS_SETUP}"
source "${WS_SETUP}"
echo "Waiting 5s for nodes to come up before publishing /start..."
sleep 5
ros2 topic pub --once /start std_msgs/msg/Bool "{data: true}"
echo
echo "[pane] /start published, dropping to shell"
exec zsh
'
EOF
)

"${KITTY_CMD[@]}" launch \
  --type=window \
  --location=vsplit \
  --cwd="$WS_DIR" \
  --title="turret-scanner" \
  zsh -i -c "$SCANNER_CMD"

"${KITTY_CMD[@]}" launch \
  --type=window \
  --location=hsplit \
  --cwd="$WS_DIR" \
  --title="turret-turret" \
  zsh -i -c "$TURRET_CMD"

"${KITTY_CMD[@]}" launch \
  --type=window \
  --location=vsplit \
  --cwd="$WS_DIR" \
  --title="turret-trigger" \
  zsh -i -c "$TRIGGER_CMD"

"${KITTY_CMD[@]}" launch \
  --type=window \
  --location=hsplit \
  --cwd="$WS_DIR" \
  --title="turret-start-pub" \
  zsh -i -c "$START_PUB_CMD"

echo "Opened scanner / turret / trigger / start-pub splits. Launching turret_vision in this pane..."

# Main launch runs in the current pane (this terminal). Setup files are zsh
# scripts, so run under zsh -- exec replaces this script's process with zsh.
exec zsh -i -c "
source \"${ROS_SETUP}\"
source \"${WS_SETUP}\"
exec ros2 launch turret_vision turret_vision.launch.py
"
