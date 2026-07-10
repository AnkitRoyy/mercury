#!/usr/bin/env bash
#
# launch_mercury.sh
#
# Splits the current kitty window once (vertical split):
#   - current pane: sources ROS setup, then runs `ros2 launch mission mission.launch.py`
#   - new pane:      sources ROS setup, then runs `ros2 launch watchdog_monitor dashboard.launch.py`
#
# Requirements:
#   - kitty terminal with `allow_remote_control yes` set in kitty.conf,
#     with kitty fully restarted after that setting was added
#   - Run from your ROS 2 workspace root, so install/setup.zsh resolves
#     relative to your current directory

set -euo pipefail

# Resolve workspace root as the directory this script is run from.
WS_DIR="$(pwd)"

ROS_SETUP="/opt/ros/jazzy/setup.zsh"
WS_SETUP="${WS_DIR}/install/setup.zsh"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "Error: $ROS_SETUP not found." >&2
    exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
    echo "Error: $WS_SETUP not found. Run this script from your workspace root (the one containing install/)." >&2
    exit 1
fi

if ! command -v kitty >/dev/null 2>&1; then
    echo "Error: kitty not found in PATH." >&2
    exit 1
fi

# Figure out how to reach kitty's remote-control socket.
# kitty auto-exports KITTY_LISTEN_ON into windows it spawns (it's typically a
# PID-scoped path like unix:/tmp/kitty-XXXX, not necessarily the static path
# from listen_on in kitty.conf). Trust it if set; otherwise fall back to the
# static path and let `kitty @` itself report a clear connection error.
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

# Watchdog dashboard runs in the new split pane.
WATCHDOG_CMD=$(cat <<EOF
zsh -i -c '
source "${ROS_SETUP}"
source "${WS_SETUP}"
ros2 launch watchdog_monitor dashboard.launch.py
echo
echo "[pane] process exited, dropping to shell"
exec zsh
'
EOF
)

# Single vertical split (new pane beside the current one) running the watchdog dashboard.
"${KITTY_CMD[@]}" launch \
    --type=window \
    --location=vsplit \
    --cwd="$WS_DIR" \
    --title="mercury-watchdog" \
    zsh -i -c "$WATCHDOG_CMD"

echo "Opened watchdog dashboard in new split. Launching mission in this pane..."

# Mission runs in the current pane (this terminal). The setup files are zsh
# scripts (they use zsh-only syntax like ${(%):-%N}), so we must run them
# under zsh, not bash -- exec replaces this script's process with zsh.
exec zsh -i -c "
source \"${ROS_SETUP}\"
source \"${WS_SETUP}\"
exec ros2 launch mission mission.launch.py
"