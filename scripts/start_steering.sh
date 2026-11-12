#!/usr/bin/env bash
set -euo pipefail

PI_A="ubuntu@192.168.50.242"
PI_B="me813@192.168.50.228"
PORT="/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"

echo "=== Starting Steering System ==="

start_pi() {
    local host="$1"
    local side="$2"
    local ids="$3"

    echo "Checking $host ..."

    ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" \
        bash -s -- "$side" "$ids" "$PORT" <<'REMOTE'
set -euo pipefail

side="$1"
ids="$2"
port="$3"
dir="$HOME/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge"

if [ "$side" = "side_a" ]; then
    script="bridge_sidea.py"
else
    script="bridge_sideb.py"
fi

test -f "$dir/$script" || {
    echo "ERROR: Bridge script not found"
    exit 1
}

test -e "$port" || {
    echo "ERROR: UART device not found"
    exit 1
}

if tmux has-session -t steering_bridge 2>/dev/null; then
    echo "Existing tmux session found"
    exit 0
fi

# Refuse to start if another bridge is already using UART.
if fuser "$port" >/dev/null 2>&1; then
    echo "ERROR: UART already in use"
    exit 1
fi

tmux new-session -d -s steering_bridge \
    "source /opt/ros/humble/setup.bash; \
     export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0; \
     exec python3 '$dir/$script' \
     --ros-args \
     -r __node:=bridge_${side} \
     -p node_ids:=$ids \
     -p port:=$port"

sleep 2

tmux has-session -t steering_bridge
echo "Started $side"
REMOTE
}

start_pi "$PI_A" "side_a" "1,2"
start_pi "$PI_B" "side_b" "3,4"

echo "Starting V5 on NUC ..."

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
V5="$PROJECT_ROOT/src/dual_steering_motor_bridge/dual_steering_ros2_bridge_V5.py"

test -f "$V5" || {
    echo "ERROR: V5 script not found"
    exit 1
}

if tmux has-session -t steering_v5 2>/dev/null; then
    echo "V5 session already exists"
elif pgrep -af '[p]ython3 .*dual_steering_ros2_bridge_V5.py' >/dev/null; then
    echo "ERROR: V5 already running outside tmux"
    exit 1
else
    tmux new-session -d -s steering_v5 \
        "source /opt/ros/humble/setup.bash; \
         export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0; \
         exec python3 '$V5'"
fi

sleep 2
tmux has-session -t steering_v5

echo "=== Startup commands completed ==="
echo "Verify ROS 2 nodes and motor status before use."
