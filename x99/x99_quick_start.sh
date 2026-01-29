#!/bin/bash
# Quick start script for X99 SLAM Server

echo "========================================"
echo "  X99 SLAM Server Setup"
echo "========================================"
echo ""

# Check ROCm
if command -v rocm-smi &> /dev/null; then
    echo "[OK] ROCm detected"
    rocm-smi -d 0 --showproductname 2>/dev/null | head -5
else
    echo "[WARNING] ROCm not detected, will use CPU"
fi

# Get local IP
echo ""
echo "Server IP addresses:"
hostname -I

LOCAL_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "Main IP: $LOCAL_IP"

# Check ports
echo ""
echo "Checking ports availability..."
for port in 9001 9002 5000; do
    if nc -z localhost $port 2>/dev/null; then
        echo "[WARNING] Port $port already in use"
    else
        echo "[OK] Port $port available"
    fi
done

# Check if YOLOv11 model exists
if [ -f "yolov11m-seg.pt" ]; then
    echo ""
    echo "[OK] YOLOv11m-seg model found"
else
    echo ""
    echo "[INFO] YOLOv11m-seg model not found, will download on first run"
fi

# Select mode
echo ""
echo "Select mode:"
echo "1) SLAM Server (standalone, with OpenCV display)"
echo "2) Web Server (with browser interface)"
read -p "Choice [2]: " MODE_CHOICE
MODE_CHOICE=${MODE_CHOICE:-2}

# Firewall setup
echo ""
read -p "Configure firewall rules? (y/n) [y]: " SETUP_FW
SETUP_FW=${SETUP_FW:-y}

if [ "$SETUP_FW" = "y" ]; then
    echo "Setting up firewall..."
    sudo ufw allow 9001/tcp 2>/dev/null && echo "[OK] Port 9001 allowed"
    sudo ufw allow 9002/tcp 2>/dev/null && echo "[OK] Port 9002 allowed"
    sudo ufw allow 5000/tcp 2>/dev/null && echo "[OK] Port 5000 allowed"
fi

echo ""
echo "========================================"
echo "  Starting SLAM Server"
echo "========================================"
echo "Listening for Jetson Nano on:"
echo "  - Port 9001 (Left camera)"
echo "  - Port 9002 (Right camera)"

if [ "$MODE_CHOICE" = "1" ]; then
    echo "Mode: Standalone SLAM"
    echo "========================================"
    echo ""
    echo "Waiting for camera streams..."
    echo "Press 'q' in OpenCV window to quit"
    echo ""
    
    python3 x99_slam_server.py
    
else
    echo "Mode: Web Interface"
    echo "========================================"
    echo ""
    echo "Web interface will be available at:"
    echo "  http://$LOCAL_IP:5000"
    echo "  http://localhost:5000"
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""
    
    python3 x99_web_server.py
fi