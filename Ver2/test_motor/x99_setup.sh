#!/bin/bash
# Setup script for X99 Server (192.168.2.10)

echo "=========================================="
echo "X99 Tank Robot Web Controller Setup"
echo "=========================================="

# Install dependencies
echo "Installing Python dependencies..."
pip3 install flask requests

# Create directory structure
mkdir -p ~/tank_robot_controller/templates

# Copy files (adjust paths as needed)
cp x99_web_controller.py ~/tank_robot_controller/
cp templates/index.html ~/tank_robot_controller/templates/

# Create systemd service for auto-start
echo "Creating systemd service..."
sudo tee /etc/systemd/system/tank-web-controller.service > /dev/null <<EOF
[Unit]
Description=Tank Robot Web Controller
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/tank_robot_controller
ExecStart=/usr/bin/python3 /home/$USER/tank_robot_controller/x99_web_controller.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo ""
echo "Setup complete!"
echo ""
echo "To start manually:"
echo "  cd ~/tank_robot_controller"
echo "  python3 x99_web_controller.py"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo systemctl enable tank-web-controller"
echo "  sudo systemctl start tank-web-controller"
echo ""
echo "Access web interface at: http://192.168.2.10:8080"
echo "=========================================="