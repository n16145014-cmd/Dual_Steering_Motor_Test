#!/usr/bin/env bash
set -uo pipefail

PI_A="ubuntu@192.168.50.242"
PI_B="me813@192.168.50.228"

echo "======================================"
echo "   STEERING SYSTEM SHUTDOWN"
echo "======================================"
echo ""
echo "WARNING:"
echo "This script stops ROS 2 bridge processes."
echo "It does NOT safely stop motor motion."
echo ""
echo "Before continuing, verify:"
echo "  - All four motors are stationary."
echo "  - Motor drives are in a safe state."
echo "  - Hardware stopping provisions are available."
echo ""

read -r -p "Press ENTER to continue (請確認實體急停已啟用)... "

errors=0

echo ""
echo "===== Stopping NUC V5 Router ====="

if tmux has-session -t steering_v5 2>/dev/null; then
    if tmux kill-session -t steering_v5; then
        echo "[OK] V5 tmux session stopped"
    else
        echo "[ERROR] Failed to stop V5"
        errors=1
    fi
else
    echo "[INFO] V5 tmux session not found"
fi

stop_pi() {
    local label="$1"
    local host="$2"

    echo ""
    echo "===== Stopping $label ====="

    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" '
        if tmux has-session -t steering_bridge 2>/dev/null; then
            tmux kill-session -t steering_bridge || exit 1
            echo "[OK] Bridge tmux session stopped"
        else
            echo "[INFO] Bridge tmux session not found"
        fi

        if tmux has-session -t steering_bridge 2>/dev/null; then
            echo "[ERROR] Bridge session still exists"
            exit 1
        fi
    '; then
        echo "[ERROR] Could not confirm shutdown on $label"
        errors=1
    fi
}

stop_pi "RASPBERRY PI A" "$PI_A"
stop_pi "RASPBERRY PI B" "$PI_B"

echo ""
echo "======================================"

if [[ "$errors" -eq 0 ]]; then
    echo "Managed tmux shutdown completed."
    echo "Verify process and motor states."
else
    echo "Shutdown incomplete: review errors."
fi

echo "======================================"

exit "$errors"
