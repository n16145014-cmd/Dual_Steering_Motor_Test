#!/usr/bin/env bash

PI_A="ubuntu@192.168.50.242"
PI_B="me813@192.168.50.228"

echo "======================================"
echo "   STEERING SYSTEM STATUS"
echo "======================================"

check_pi() {
    local label="$1"
    local host="$2"

    echo ""
    echo "===== $label ====="

    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" true; then
        echo "[ERROR] SSH connection failed"
        return
    fi

    echo "[OK] SSH connected"

    ssh -o BatchMode=yes "$host" '
        if tmux has-session -t steering_bridge 2>/dev/null; then
            echo "[OK] tmux steering_bridge exists"
        else
            echo "[WARN] tmux steering_bridge missing"
        fi

        if pgrep -f "[p]ython3 .*bridge_side[ab].py" >/dev/null; then
            echo "[OK] Bridge process running"
        else
            echo "[WARN] Bridge process not running"
        fi

        if fuser /dev/ttyUSB0 >/dev/null 2>&1; then
            echo "[INFO] UART is in use"
        else
            echo "[INFO] UART is not in use"
        fi
    '
}

check_pi "RASPBERRY PI A - M1/M2" "$PI_A"
check_pi "RASPBERRY PI B - M3/M4" "$PI_B"

echo ""
echo "===== NUC - V5 ROUTER ====="

if tmux has-session -t steering_v5 2>/dev/null; then
    echo "[OK] tmux steering_v5 exists"
else
    echo "[WARN] tmux steering_v5 missing"
fi

if pgrep -f '[p]ython3 .*dual_steering_ros2_bridge_V5.py' >/dev/null; then
    echo "[OK] V5 Router process running"
else
    echo "[WARN] V5 Router process not running"
fi

echo ""
echo "===== ROS 2 NODES ====="

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

nodes=$(timeout 8 ros2 node list 2>&1)

for node in \
    /bridge_side_a \
    /bridge_side_b \
    /four_motor_router_v5
do
    if printf '%s\n' "$nodes" | grep -Fxq "$node"; then
        echo "[OK] $node"
    else
        echo "[WARN] $node not discovered"
    fi
done

echo ""
echo "======================================"
echo "Status check complete"
echo "======================================"
