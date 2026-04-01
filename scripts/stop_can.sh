#!/bin/bash
# stop_can.sh - CAN インターフェースを停止
set -e

INTERFACE="${CAN_IFACE:-can0}"

echo "Stopping $INTERFACE..."
sudo ip link set "$INTERFACE" down 2>/dev/null || true
sudo killall slcand 2>/dev/null || true
echo "Done."
