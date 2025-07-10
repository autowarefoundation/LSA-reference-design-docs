# Software Configuration for ARM-based ECUs

This guide provides comprehensive instructions for deploying Autoware on ARM-based ECUs, with specific focus on NVIDIA Jetson and AGX Orin platforms for Low Speed Autonomy vehicles.

## System Preparation

### CUDA Toolkit Installation

#### For NVIDIA AGX Orin

**Important**: Do NOT install CUDA packages manually on AGX Orin as it may break the system. CUDA is included with JetPack and must be installed through NVIDIA's SDK Manager.

**Installation Steps:**

1. **Download NVIDIA SDK Manager** from [https://developer.nvidia.com/nvidia-sdk-manager](https://developer.nvidia.com/nvidia-sdk-manager)

2. **Select JetPack 6.0** (or latest version) in SDK Manager
   - This will flash the Orin device and install the appropriate CUDA version
   - JetPack 6.0 includes CUDA 12.2, cuDNN, TensorRT, and other essential libraries

3. **Flash and Install**
   - Connect your AGX Orin to the host PC via USB-C
   - Follow SDK Manager prompts to flash the device
   - The process will install Ubuntu, CUDA, and all necessary drivers

4. **Verify Installation** after flashing:
```bash
# Check CUDA version
nvcc --version

# Verify Jetson platform and monitor system
sudo apt install -y python3-pip
pip3 install jetson-stats
sudo jtop

# Check JetPack version
cat /etc/nv_tegra_release
```

#### For Other ARM64 Platforms

For ARM64 platforms other than NVIDIA Jetson:
- Check the manufacturer's product specifications or manual for CUDA support
- Most non-NVIDIA ARM platforms do not support CUDA
- Consider using CPU-only or other acceleration options if CUDA is not available

**Supported JetPack Versions by Platform:**
- AGX Orin: JetPack 6.0 or later (recommended)
- Xavier Series: JetPack 5.1 or later
- Nano/TX2: Check NVIDIA's compatibility matrix

## Platform-Specific Configuration

### Jetson-Specific System Setup

```bash
# Install Jetson-specific packages
sudo apt install -y \
  nvidia-jetpack \
  nvidia-l4t-tools \
  nvidia-l4t-multimedia \
  nvidia-l4t-camera

# Configure system for development
sudo nvpmodel -q  # Query available power modes
sudo jetson_clocks  # Set clocks to maximum (for development)
```

### Memory Configuration

ARM platforms have unified memory architecture:

```bash
# Configure memory for GPU applications
echo "export TF_FORCE_GPU_ALLOW_GROWTH=true" >> ~/.bashrc
echo "export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128" >> ~/.bashrc
source ~/.bashrc
```

## Verification

### System Check Script

Create a verification script for ARM systems:

```bash
cat > ~/verify_arm_setup.sh << 'EOF'
#!/bin/bash

echo "=== ARM Autoware Setup Verification ==="
echo ""

echo "1. System Information"
echo "   OS: $(lsb_release -d | cut -f2)"
echo "   Kernel: $(uname -r)"
echo "   Architecture: $(uname -m)"
echo "   RAM: $(free -h | grep Mem | awk '{print $2}')"

# Check if running on Jetson
if [ -f /etc/nv_tegra_release ]; then
    echo "   Platform: NVIDIA Jetson"
    echo "   L4T Version: $(head -n 1 /etc/nv_tegra_release)"
fi
echo ""

echo "2. GPU Status"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
elif command -v tegrastats &> /dev/null; then
    echo "   Jetson GPU detected (use jtop for details)"
fi

if command -v nvcc &> /dev/null; then
    echo "   CUDA: $(nvcc --version | grep release | awk '{print $6}')"
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

chmod +x ~/verify_arm_setup.sh
```

Run the verification:

```bash
~/verify_arm_setup.sh
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

# Launch minimal Autoware (simulation mode)
ros2 launch autoware_launch logging_simulator.launch.xml
```

## Development Workflows

### Native Development on Jetson

Set up a development workspace:

```bash
# Create workspace
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws

# Clone Autoware repositories
git clone https://github.com/autowarefoundation/autoware.universe.git src/universe

# Install dependencies
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# Build (this may take time on ARM)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Cross-Compilation (Optional)

For faster builds, consider cross-compilation from an x86 host. See [Docker Cross-Compilation Guide](https://docs.docker.com/build/building/multi-platform/) for details.

### Containerized Development

For team collaboration and reproducible environments, see our [Containerized Development Guide](containerized-development.md).

## Monitoring and Debugging

### Jetson-Specific Monitoring

```bash
# Real-time system monitoring
sudo jtop

# GPU/CPU/Memory stats
tegrastats

# Temperature monitoring
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

### ROS 2 Monitoring

```bash
# Monitor topics
ros2 topic list
ros2 topic hz /sensing/lidar/concatenated/pointcloud

# Check node status
ros2 node list
ros2 node info /perception/lidar_centerpoint

# Analyze TF tree
ros2 run tf2_tools view_frames
```

## Troubleshooting

### Common Issues

1. **CUDA Out of Memory**
   ```bash
   # Check memory usage
   tegrastats | grep RAM
   
   # Enable memory growth
   export TF_FORCE_GPU_ALLOW_GROWTH=true
   ```

2. **JetPack Version Mismatch**
   ```bash
   # Check installed version
   cat /etc/nv_tegra_release
   
   # Ensure packages match JetPack version
   dpkg -l | grep nvidia-l4t
   ```

3. **Performance Issues**
   ```bash
   # Set maximum performance mode
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

4. **ROS 2 Communication Issues**
   - Check firewall settings
   - Verify ROS_DOMAIN_ID is consistent
   - Ensure network interfaces are properly configured

## Platform-Specific Customizations

For advanced ARM platform customizations including:

- Ansible automation for fleet deployment
- Hardware-specific optimizations
- Security hardening for production

See: [ARM Customization Guide](customization.md)

## Next Steps

1. **Configure Sensors**: See [Sensor Configuration Guide](../sensor-configuration/index.md) for detailed sensor setup
2. **Vehicle Integration**: Configure CAN bus and vehicle interface  
3. **Middleware Setup**: Consider [RMW Zenoh](../middleware-configuration/index.md) for optimized communication
4. **Performance Tuning**: Run benchmarks and optimize for your use case

## Additional Resources

- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded-computing)
- [Autoware ARM Support](https://autowarefoundation.github.io/autoware-documentation/)
- [ROS 2 on ARM Guide](https://docs.ros.org/en/humble/How-To-Guides/Cross-compilation.html)
- [JetPack SDK Documentation](https://docs.nvidia.com/jetson/jetpack/)
