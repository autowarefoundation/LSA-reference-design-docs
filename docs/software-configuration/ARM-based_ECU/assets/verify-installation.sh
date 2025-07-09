#!/bin/bash

echo "=== System Verification ==="
echo "OS Version: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo ""

echo "=== CUDA Verification ==="
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
else
    echo "NVIDIA driver not found"
fi

if command -v nvcc &> /dev/null; then
    echo "CUDA Version: $(nvcc --version | grep release | awk '{print $6}')"
else
    echo "CUDA not found"
fi
echo ""

echo "=== ROS 2 Verification ==="
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Distro: $ROS_DISTRO"
    echo "ROS 2 Version: $(ros2 --version 2>&1 | grep '^ros2')"
else
    echo "ROS 2 not found"
fi
echo ""

echo "=== Autoware Verification ==="
if [ -f /opt/autoware/autoware-env ]; then
    source /opt/autoware/autoware-env
    echo "Autoware packages installed: $(ros2 pkg list | grep -c autoware)"
else
    echo "Autoware not found"
fi
