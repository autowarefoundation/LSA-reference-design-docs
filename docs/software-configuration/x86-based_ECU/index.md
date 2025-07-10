# Software Configuration for x86-based ECUs

This guide provides comprehensive instructions for deploying Autoware on x86-based ECUs (Intel/AMD platforms) for Low Speed Autonomy vehicles.

## System Preparation

### Operating System Installation

Follow the base [Getting Started Guide](../getting-started/index.md) for Ubuntu 22.04 installation, then apply x86-specific configurations:

```bash
# Install x86-specific packages
sudo apt install -y \
  intel-microcode \
  amd64-microcode \
  linux-tools-generic \
  cpufrequtils \
  i7z \
  powertop
```

## CUDA and GPU Configuration

### 1. CUDA Toolkit Installation

For x86-based systems, CUDA must be installed manually. Visit the [NVIDIA CUDA Toolkit Archive](https://developer.nvidia.com/cuda-toolkit-archive) to find the appropriate version for your system.

**Recommended Installation Method: deb (network)**

Here's an example of installing CUDA Toolkit 12.3 using the network deb method:

```bash
# Step 1: Download and install the cuda-keyring package
# Visit https://developer.nvidia.com/cuda-toolkit-archive and select:
# - Operating System: Linux
# - Architecture: x86_64
# - Distribution: Ubuntu
# - Version: 22.04
# - Installer Type: deb (network)

# Install the keyring
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

# Step 2: Update package lists
sudo apt update

# Step 3: Install CUDA Toolkit
sudo apt install -y cuda-toolkit-12-3

# Step 4: Verify installation
nvidia-smi
nvcc --version
```

### 2. Install Additional GPU Libraries

```bash
# Install cuDNN (after CUDA toolkit is installed)
# Visit https://developer.nvidia.com/cudnn for the latest version
sudo apt install -y cudnn

# Install TensorRT
sudo apt install -y tensorrt
```

### 3. Configure GPU Environment

```bash
# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify GPU is accessible
nvidia-smi
```

## Verification

### System Check Script

Create a comprehensive verification script:

```bash
cat > ~/verify_x86_setup.sh << 'EOF'
#!/bin/bash

echo "=== x86 Autoware Setup Verification ==="
echo ""

echo "1. System Information"
echo "   OS: $(lsb_release -d | cut -f2)"
echo "   Kernel: $(uname -r)"
echo "   CPU: $(lscpu | grep "Model name" | cut -d':' -f2 | xargs)"
echo "   RAM: $(free -h | grep Mem | awk '{print $2}')"
echo ""

echo "2. GPU Status"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    echo "   CUDA: $(nvcc --version | grep release | awk '{print $6}')"
else
    echo "   No NVIDIA GPU detected"
fi
echo ""

echo "3. ROS 2 Environment"
source /opt/ros/humble/setup.bash
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_VERSION: $(ros2 --version 2>&1 | grep '^ros2')"
echo ""

echo "4. Autoware Installation"
if [ -f /opt/autoware/autoware-env ]; then
    source /opt/autoware/autoware-env
    echo "   Autoware packages: $(ros2 pkg list | grep -c autoware)"
    echo "   Launch files: $(ros2 pkg prefix autoware_launch >/dev/null 2>&1 && echo "Found" || echo "Not found")"
else
    echo "   Autoware not installed"
fi
echo ""

echo "5. Network Interfaces"
echo "   Available: $(ip -br link show | grep -v lo | awk '{print $1}' | tr '\n' ' ')"
echo ""

echo "Verification complete!"
EOF

chmod +x ~/verify_x86_setup.sh
```

Run the verification:

```bash
~/verify_x86_setup.sh
```

### Test Autoware Components

```bash
# Source ROS 2 and Autoware
source /opt/ros/humble/setup.bash
source /opt/autoware/autoware-env

# Test ROS 2 communication
ros2 doctor

# List Autoware packages
ros2 pkg list | grep autoware

# Check TF tree (in simulation)
ros2 run tf2_tools view_frames
```

## Troubleshooting

### Common Issues

1. **CUDA Version Mismatch**
   ```bash
   # Check NVIDIA driver and CUDA compatibility
   nvidia-smi
   # Ensure driver version supports your CUDA version
   ```

2. **ROS 2 Package Conflicts**
   ```bash
   # Clean and rebuild if needed
   cd ~/autoware_ws
   rm -rf build install log
   colcon build --symlink-install
   ```

3. **GPU Memory Errors**
   ```bash
   # Monitor GPU memory usage
   watch -n 1 nvidia-smi
   
   # Set environment variable to allow memory growth
   export TF_FORCE_GPU_ALLOW_GROWTH=true
   ```

4. **Network Configuration Issues**
   - Verify network interfaces are properly configured
   - Check firewall settings for ROS 2 DDS communication
   - Ensure multicast is enabled for DDS discovery

## Performance Monitoring

### System Monitoring Tools

```bash
# CPU and system monitoring
htop

# GPU monitoring
nvtop

# ROS 2 specific monitoring
ros2 run rqt_top rqt_top

# Network monitoring
iftop
```

### ROS 2 Performance Analysis

```bash
# Monitor topic frequencies
ros2 topic hz /sensing/lidar/concatenated/pointcloud

# Check node CPU usage
ros2 run rqt_top rqt_top

# Analyze communication graph
ros2 run rqt_graph rqt_graph
```

## Next Steps

1. **Configure Sensors**: See [Sensor Configuration Guide](../sensor-configuration/index.md) for detailed sensor setup
2. **Vehicle Integration**: Configure CAN bus and vehicle interface
3. **Calibration**: Run sensor calibration procedures
4. **Testing**: Perform system integration tests

## Additional Resources

- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA CUDA Documentation](https://docs.nvidia.com/cuda/)
- [x86 Platform Optimization Guide](https://www.intel.com/content/www/us/en/developer/overview.html)
