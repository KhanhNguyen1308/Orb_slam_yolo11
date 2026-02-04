#!/bin/bash
pip3 install opencv-python>=4.8.0
pip3 install opencv-contrib-python>=4.8.0
pip3 install numpy>=1.24.0
pip3 install flask>=2.3.0
pip3 install flask-socketio>=5.3.0
pip3 install python-socketio>=5.9.0
# YOLO segmentation
pip3 install ultralytics>=8.0.0
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/rocm6.1
# Optional: For better performance
pip3 install eventlet>=0.33.0

# ROCm support for Radeon MI50 (optional, uncomment if using AMD GPU)
#torch-rocm>=2.0.0

# Additional utilities
pip3 install Pillow>=10.0.0
pip3 install scipy>=1.11.0