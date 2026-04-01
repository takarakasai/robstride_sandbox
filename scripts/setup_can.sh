#!/bin/bash
# setup_can.sh - /dev/ttyUSB0 (SLCAN) を SocketCAN インターフェース (can0) として設定
#
# Usage:
#   sudo ./setup_can.sh          # デフォルト: 1Mbps
#   sudo ./setup_can.sh 500000   # 500kbps

set -e

DEVICE="${CAN_DEVICE:-/dev/ttyACM1}"
INTERFACE="${CAN_IFACE:-can0}"
BITRATE="${1:-1000000}"

# SLCAN ビットレートコード:
#   s0 = 10kbps,   s1 = 20kbps,   s2 = 50kbps
#   s3 = 100kbps,  s4 = 125kbps,  s5 = 250kbps
#   s6 = 500kbps,  s7 = 800kbps,  s8 = 1000kbps
case "$BITRATE" in
    10000)   SLCAN_SPEED=0 ;;
    20000)   SLCAN_SPEED=1 ;;
    50000)   SLCAN_SPEED=2 ;;
    100000)  SLCAN_SPEED=3 ;;
    125000)  SLCAN_SPEED=4 ;;
    250000)  SLCAN_SPEED=5 ;;
    500000)  SLCAN_SPEED=6 ;;
    800000)  SLCAN_SPEED=7 ;;
    1000000) SLCAN_SPEED=8 ;;
    *)
        echo "Error: Unsupported bitrate: $BITRATE"
        echo "Supported: 10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000"
        exit 1
        ;;
esac

echo "=== SLCAN CAN Interface Setup ==="
echo "  Device:    $DEVICE"
echo "  Interface: $INTERFACE"
echo "  Bitrate:   $BITRATE bps (SLCAN code: s$SLCAN_SPEED)"
echo ""

# 既存のインターフェースを停止
if ip link show "$INTERFACE" &>/dev/null; then
    echo "Stopping existing $INTERFACE..."
    ip link set "$INTERFACE" down 2>/dev/null || true
    killall slcand 2>/dev/null || true
    sleep 1
fi

# シリアルポートの設定
echo "Configuring serial port $DEVICE..."
stty -F "$DEVICE" speed 115200 raw -echo -echoe -echok

# slcand でデーモン起動
echo "Starting slcand (speed=$SLCAN_SPEED)..."
slcand -o -s"$SLCAN_SPEED" -t hw -S 115200 "$DEVICE" "$INTERFACE"
sleep 1

# インターフェース起動
echo "Bringing up $INTERFACE..."
ip link set "$INTERFACE" up
sleep 0.5

# 確認
echo ""
echo "=== Interface Status ==="
ip -details link show "$INTERFACE"
echo ""
echo "Done! $INTERFACE is ready."
echo ""
echo "Test commands:"
echo "  candump $INTERFACE          # CAN トラフィック監視"
echo "  cansend $INTERFACE 001#DEADBEEF  # テストフレーム送信"
echo ""
echo "To stop:"
echo "  sudo ip link set $INTERFACE down && sudo killall slcand"
