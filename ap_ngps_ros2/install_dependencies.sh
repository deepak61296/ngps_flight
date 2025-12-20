#!/bin/bash

echo "Installing NGPS ROS2 dependencies..."

if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 is not installed. Please install ROS2 Humble first."
    exit 1
fi

echo "Installing ROS2 packages..."
sudo apt update
sudo apt install -y \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport

echo "Installing Python dependencies..."
pip install -r requirements.txt

if [ ! -d "LightGlue" ]; then
    echo "Cloning LightGlue repository..."
    git clone https://github.com/snktshrma272/LightGlue.git
    cd LightGlue
    pip install -e .
    cd ..
fi

echo "Dependencies installed successfully!"
echo ""
echo "To build the package:"
echo "cd /path/to/your/workspace"
echo "colcon build --packages-select ap_ngps_ros2"
echo "source install/setup.bash"
echo ""
echo "To run the node:"
echo "ros2 launch ap_ngps_ros2 ngps_localization.launch.py"
