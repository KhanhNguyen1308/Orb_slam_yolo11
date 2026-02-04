#!/bin/bash
# Setup script for Jetson Nano (192.168.2.13)

echo "=========================================="
echo "Jetson Nano Serial Bridge Setup"
echo "=========================================="

# Install dependencies
echo "Installing Python dependencies..."
pip3 install flask pyserial

# Create directory
mkdir -p ~/jetson_serial_bridge

# Copy file
cp jetson_serial_bridge.py ~/jetson_serial_bridge/

# Add user to dialout group for serial access
echo "Adding $USER to dialout group..."
sudo usermod -a -G dialout $USER

# Find RP2040 serial port
echo ""
echo "Detecting serial ports..."
echo "Available ports:"
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"

echo ""
echo "If your RP2040 is not at /dev/ttyACM0, edit jetson_serial_bridge.py"
echo "and change SERIAL_PORT to the correct device."

# Create systemd service
echo ""
echo "Creating systemd service..."
sudo tee /etc/systemd/system/jetson-serial-bridge.service > /dev/null <<EOF
[Unit]
Description=Jetson Nano Serial Bridge for Tank Robot
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/jetson_serial_bridge
ExecStart=/usr/bin/python3 /home/$USER/jetson_serial_bridge/jetson_serial_bridge.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo ""
echo "Setup complete!"
echo ""
echo "IMPORTANT: Log out and back in for group changes to take effect"
echo ""
echo "To start manually:"
echo "  cd ~/jetson_serial_bridge"
echo "  python3 jetson_serial_bridge.py"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo systemctl enable jetson-serial-bridge"
echo "  sudo systemctl start jetson-serial-bridge"
echo ""
echo "Test connection:"
echo "  curl http://192.168.2.13:5000/health"
echo "=========================================="