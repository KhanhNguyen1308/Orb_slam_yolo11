#!/bin/bash
# Quick start script for Jetson Nano Camera Streaming

echo "========================================"
echo "  Jetson Nano Camera Streamer Setup"
echo "========================================"
echo ""

# Check if running on Jetson
if [ -f /etc/nv_tegra_release ]; then
    echo "[OK] Running on Jetson Nano"
else
    echo "[WARNING] Not detected as Jetson (continuing anyway)"
fi

# Check cameras
echo ""
echo "Detecting cameras..."
ls /dev/video* 2>/dev/null || echo "[WARNING] No /dev/video* devices found"

# Python camera detection
python3 << 'EOF'
import cv2
cameras = [i for i in range(10) if cv2.VideoCapture(i).isOpened()]
if cameras:
    print(f"[OK] Found cameras at IDs: {cameras}")
else:
    print("[ERROR] No cameras detected!")
EOF

# Get server IP
echo ""
read -p "Enter X99 server IP [192.168.1.100]: " SERVER_IP
SERVER_IP=${SERVER_IP:-192.168.1.100}

# Test connectivity
echo ""
echo "Testing connection to $SERVER_IP..."
ping -c 2 $SERVER_IP > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "[OK] Server is reachable"
else
    echo "[ERROR] Cannot reach server at $SERVER_IP"
    exit 1
fi

# Get camera IDs
echo ""
read -p "Left camera ID [0]: " LEFT_CAM
LEFT_CAM=${LEFT_CAM:-0}

read -p "Right camera ID [1]: " RIGHT_CAM
RIGHT_CAM=${RIGHT_CAM:-1}

# Resolution
echo ""
echo "Select resolution:"
echo "1) 320x240 (Low, fastest)"
echo "2) 640x480 (Medium, recommended)"
echo "3) 1280x720 (High, slower)"
read -p "Choice [2]: " RES_CHOICE
RES_CHOICE=${RES_CHOICE:-2}

case $RES_CHOICE in
    1)
        WIDTH=320
        HEIGHT=240
        ;;
    2)
        WIDTH=640
        HEIGHT=480
        ;;
    3)
        WIDTH=1280
        HEIGHT=720
        ;;
    *)
        WIDTH=640
        HEIGHT=480
        ;;
esac

# JPEG quality
read -p "JPEG quality (60-95) [80]: " QUALITY
QUALITY=${QUALITY:-80}

# Enable max performance
echo ""
echo "Enabling maximum performance mode..."
sudo nvpmodel -m 0 2>/dev/null && echo "[OK] Max power mode enabled" || echo "[SKIP] nvpmodel not available"
sudo jetson_clocks 2>/dev/null && echo "[OK] Max clocks enabled" || echo "[SKIP] jetson_clocks not available"

# Start streaming
echo ""
echo "========================================"
echo "  Starting Camera Stream"
echo "========================================"
echo "Server: $SERVER_IP"
echo "Left camera: $LEFT_CAM → Port 9001"
echo "Right camera: $RIGHT_CAM → Port 9002"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "Quality: $QUALITY"
echo "========================================"
echo ""
echo "Press Ctrl+C to stop"
echo ""

python3 jetson_camera_client.py \
    --server $SERVER_IP \
    --left-camera $LEFT_CAM \
    --right-camera $RIGHT_CAM \
    --width $WIDTH \
    --height $HEIGHT \
    --quality $QUALITY