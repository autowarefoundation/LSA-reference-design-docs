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

### Memory Management

ARM platforms have unified memory architecture:

```bash
# Configure memory growth for GPU applications
echo "export TF_FORCE_GPU_ALLOW_GROWTH=true" >> ~/.bashrc
echo "export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128" >> ~/.bashrc

# Optimize memory bandwidth
sudo echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked
sudo echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/state
```

### Thermal Management

```yaml
# ansible/roles/jetson_thermal/tasks/main.yml
---
- name: Configure thermal zones
  lineinfile:
    path: /etc/nvfancontrol.conf
    regexp: '^FAN_PROFILE'
    line: 'FAN_PROFILE quiet'
    
- name: Set temperature thresholds
  copy:
    content: |
      <FAN_PROFILE name="quiet">
        <FAN_CONTROL channel="0">
          <TEMP_POINT temp="50" speed="0"/>
          <TEMP_POINT temp="60" speed="80"/>
          <TEMP_POINT temp="70" speed="120"/>
          <TEMP_POINT temp="80" speed="255"/>
        </FAN_CONTROL>
      </FAN_PROFILE>
    dest: /etc/nvfancontrol.d/autoware.conf
```

## Sensor Configuration
The following instrucitons configure the sensor for LSA vehicles.

### CSI Camera Configuration

```bash
# Configure CSI cameras
sudo apt install -y nvidia-l4t-camera

# Verify camera detection
v4l2-ctl --list-devices

# Test camera feed
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv flip-method=0 ! \
  'video/x-raw,format=BGRx' ! \
  videoconvert ! \
  'video/x-raw,format=BGR' ! \
  fakesink
```

### Hardware Accelerated Processing

```python
# Example: Hardware-accelerated image processing
import cv2

def create_hw_accelerated_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink"
    )

cap = cv2.VideoCapture(create_hw_accelerated_pipeline(), cv2.CAP_GSTREAMER)
```

## Autoware Deployment

### 1. Core Autoware Installation

Complete the general setup from [Deployment Setup](../deployment-setup/#autoware-deployment-via-debian-packages), then proceed with ARM-specific configuration. 

## Autoware Deployment via Debian Packages
(This part should be moved to ECU-dependent deployment.)

#### 1.1 Configure Autoware APT Repository

```bash
# Download and install repository configuration
wget https://github.com/autowarefoundation/autoware/releases/latest/download/autoware-apt-config.deb
sudo dpkg -i autoware-apt-config.deb

# Update package lists
sudo apt update
```

#### 1.2. Deploy Autoware Core Packages

```bash
# Install complete Autoware stack
sudo apt install -y autoware-universe

# Or install specific components
sudo apt install -y \
  autoware-common \
  autoware-control \
  autoware-localization \
  autoware-perception \
  autoware-planning
```

#### 1.3. Configure Environment

```bash
# Add Autoware setup to bashrc
echo "source /opt/autoware/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 pkg list | grep autoware
```

### 2. ARM-Specific Dependencies

The following instructions install ARM-specific tools before deploying Autoware.

```bash
# Install ARM-optimized libraries
sudo apt install -y \
  libopenblas-dev \
  liblapack-dev \
  libeigen3-dev \
  libboost-all-dev

# Install Jetson-specific tools
sudo apt install -y \
  nvidia-jetpack \
  nvidia-l4t-tools \
  nvidia-l4t-multimedia
```

### 3. Configure for Jetson

Create Jetson-optimized configuration:

```yaml
# ~/autoware_config/jetson_optimization.yaml
/**:
  ros__parameters:
    # DLA acceleration
    use_dla: true
    dla_core: 0
    
    # GPU configuration
    gpu_id: 0
    allow_gpu_memory_growth: true
    
    # TensorRT optimization
    use_tensorrt: true
    tensorrt_precision: "INT8"  # Jetson supports INT8
    tensorrt_workspace_size: 1073741824  # 1GB
    
    # CPU configuration
    cpu_cores: 8
    enable_cpu_affinity: true
```

You can also download the [file](assets/jetson_optimization.yaml) and move it 'autoware_config'. 

### 4. Launch Configuration

```xml
<!-- autoware_jetson.launch.xml -->
<launch>
  <!-- Load Jetson-specific parameters -->
  <include file="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml">
    <arg name="vehicle_model" value="jetson_vehicle"/>
    <arg name="sensor_model" value="jetson_sensor_kit"/>
    <arg name="use_tensorrt" value="true"/>
    <arg name="use_dla" value="true"/>
  </include>
  
  <!-- Jetson-specific nodes -->
  <node pkg="jetson_stats_publisher" exec="jtop_publisher" name="jetson_monitor"/>
</launch>
```

Download the [lauch file](assets/autoware_jetson.launch.xml).

system. 


#### 5.1. System Check Script

The first step is to create a verification script:

```bash
cat > verify-installation.sh << 'EOF'
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
if [ -f /opt/autoware/setup.bash ]; then
    source /opt/autoware/setup.bash
    echo "Autoware packages installed: $(ros2 pkg list | grep -c autoware)"
else
    echo "Autoware not found"
fi
EOF
```
You can also download the [file](assets/verify-installation.sh) and move the shell script to the working directory.

The last step executes the verification script to verify the deployment. 

```
chmod +x verify-installation.sh
./verify-installation.sh
```

#### 5.2. Test Basic Functionality

```bash
# Test ROS 2 communication
ros2 doctor

# Launch minimal Autoware nodes
ros2 launch autoware_launch logging_simulator.launch.xml
```

## Development Workflows

### Native Development

To directly develope on Jetson, you need to install the tools and set up the workspace.

```bash
# Install development tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions

# Setup workspace
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws
git clone https://github.com/autowarefoundation/autoware.universe.git src/universe
```

### Containerized Development

For reproducible environments, see our comprehensive [Containerized Development Guide](containerized-development.md) which covers:

- Docker setup for Jetson
- Cross-compilation from x86
- Automated build pipelines
- Team collaboration workflows

## Performance Enhancement

To enhance the performance of the system, one may use the following configuration for power consumption, DLA acceleration, and memory bandwidth optimization.

### Power Consumption Mode Configuration

```bash
# View available power modes
sudo nvpmodel -q

# Set power mode (example for AGX Orin)
sudo nvpmodel -m 0  # MAXN (maximum performance)
sudo nvpmodel -m 1  # 50W mode
sudo nvpmodel -m 2  # 30W mode
sudo nvpmodel -m 3  # 15W mode

# Lock clocks to maximum
sudo jetson_clocks --fan
```

### Deep Learning Accelerator (DLA) 

```cpp
// Example: Configure DLA in TensorRT
#include <NvInfer.h>

void configureDLA(nvinfer1::IBuilder* builder) {
    auto config = builder->createBuilderConfig();
    
    // Enable DLA
    config->setFlag(BuilderFlag::kGPU_FALLBACK);
    config->setDefaultDeviceType(DeviceType::kDLA);
    
    // Use DLA core 0
    config->setDLACore(0);
    
    // Set INT8 precision for DLA
    config->setFlag(BuilderFlag::kINT8);
}
```

### Memory Bandwidth Optimization

```bash
# Monitor memory bandwidth
sudo tegrastats

# Optimize EMC frequency
sudo echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked
sudo cat /sys/kernel/debug/bpmp/debug/clk/emc/max_rate > \
  /sys/kernel/debug/bpmp/debug/clk/emc/rate
```

## Tools for Performance Evaluation

### Monitoring Tools

```bash
# Real-time system monitoring
sudo jtop

# Detailed GPU metrics
sudo tegrastats --interval 1000

# Power consumption
sudo cat /sys/bus/i2c/drivers/ina3221x/1-0040/iio_device/in_power0_input
```

### Expected Performance Metrics for ARM-based ECUs

| Component | AGX Orin | Xavier | Orin Nano |
|-----------|----------|---------|-----------|
| LiDAR Detection | 10 Hz | 8 Hz | 5 Hz |
| Camera Detection | 20 Hz | 15 Hz | 10 Hz |
| Planning | 10 Hz | 10 Hz | 8 Hz |
| Control | 50 Hz | 50 Hz | 50 Hz |
| Power Usage | 40W | 25W | 12W |

### Profiling with Nsight

```bash
# System-wide profiling
sudo /opt/nvidia/nsight-systems/2023.3.1/bin/nsys profile \
  -t cuda,nvtx,osrt,cudnn,cublas \
  -o autoware_profile \
  ros2 launch autoware_launch autoware.launch.xml

# Analyze results
nsys-ui autoware_profile.nsys-rep
```

## Troubleshooting

### Common Issues

1. **CUDA Out of Memory**
```bash
# Monitor GPU memory
watch -n 1 nvidia-smi

# Solution: Enable memory growth
export TF_FORCE_GPU_ALLOW_GROWTH=true
```

2. **Thermal Throttling**
```bash
# Check thermal status
sudo cat /sys/devices/virtual/thermal/thermal_zone*/type
sudo cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Solution: Improve cooling or reduce power mode
sudo nvpmodel -m 2  # Lower power mode
```

3. **DLA Failures**
```bash
# Check DLA status
sudo cat /sys/kernel/debug/nvdla/dla0/busy

# Solution: Fallback to GPU
# Set use_dla: false in configuration
```

## Platform-Specific Customizations

For advanced ARM platform customizations including:

- Ansible automation for fleet deployment
- Hardware-specific sensor integration
- Custom kernel module development
- Security hardening for production

One can use this guide: [ARM Customization Guide](customization.md).

## Next Steps

- Explore [Containerized Development](containerized-development.md) for team collaboration
- Configure [RMW Zenoh](../rmw_zenoh/index.md) for optimized communication
- Review [platform-specific customizations](customization.md) for production deployment
- Compare with [x86-based ECU](../x86-based_ECU/index.md) configurations
