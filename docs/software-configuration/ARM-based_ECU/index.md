# Software Configuration for ARM-based ECUs

This guide provides comprehensive instructions for deploying Autoware on ARM-based ECUs, with specific focus on NVIDIA Jetson and AGX Orin platforms for Low Speed Autonomy vehicles.

## Technical Specifications

### NVIDIA Jetson Platform Options

#### AGX Orin Series (Recommended)
- **AGX Orin 64GB Developer Kit**
  - CPU: 12-core Arm Cortex-A78AE
  - GPU: 2048-core NVIDIA Ampere with 64 Tensor Cores
  - Memory: 64GB 256-bit LPDDR5
  - Storage: 64GB eMMC, NVMe M.2 support
  - AI Performance: 275 TOPS
  - Power: 15W-60W configurable

- **AGX Orin 32GB**
  - Similar specs with 32GB memory
  - AI Performance: 200 TOPS
  - Power: 15W-40W configurable

#### Xavier Series
- **AGX Xavier Industrial**
  - CPU: 8-core NVIDIA Carmel ARM v8.2
  - GPU: 512-core Volta with 64 Tensor Cores
  - Memory: 32GB 256-bit LPDDR4x
  - AI Performance: 30 TOPS
  - Power: 10W-30W configurable

#### Orin Nano/NX Series (Entry Level)
- **Orin Nano 8GB**
  - CPU: 6-core Arm Cortex-A78AE
  - GPU: 1024-core NVIDIA Ampere
  - Memory: 8GB 128-bit LPDDR5
  - AI Performance: 40 TOPS
  - Power: 7W-15W configurable

### Industrial ARM Platforms
- **ADLINK AVA-ORIN**: Rugged AGX Orin platform for vehicles
- **Connect Tech Rogue**: Carrier board for AGX Orin with automotive interfaces
- **Auvidea JNX30**: Industrial carrier for Jetson with CAN/LIN interfaces

## System Preparation

### JetPack Installation

The NVIDIA JetPack SDK provides the foundation for ARM-based deployments:

```bash
# For AGX Orin - Use JetPack 5.1.2 or later
# Download NVIDIA SDK Manager from:
# https://developer.nvidia.com/nvidia-sdk-manager

# Or flash directly using command line:
sudo apt install nvidia-jetpack
```

### Post-Installation Setup

```bash
# Verify JetPack installation
cat /etc/nv_tegra_release

# Check CUDA version
nvcc --version

# Install Jetson utilities
sudo apt update
sudo apt install -y python3-pip
pip3 install -U jetson-stats
sudo systemctl restart jtop.service

# Configure power mode for development
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks
```

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

### 1. Follow Base Installation

Complete the general setup from [Deployment Setup](../deployment-setup/index.md), then proceed with ARM-specific configuration.

### 2. ARM-Specific Dependencies

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

## Development Workflows

### Native Development

For direct development on Jetson:

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

## Performance Optimization

### Power Mode Configuration

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

### DLA Acceleration

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

## Performance Evaluation

### Monitoring Tools

```bash
# Real-time system monitoring
sudo jtop

# Detailed GPU metrics
sudo tegrastats --interval 1000

# Power consumption
sudo cat /sys/bus/i2c/drivers/ina3221x/1-0040/iio_device/in_power0_input
```

### Expected Performance Metrics

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

See our [ARM Customization Guide](customization.md).

## Next Steps

- Explore [Containerized Development](containerized-development.md) for team collaboration
- Configure [RMW Zenoh](../rmw_zenoh/index.md) for optimized communication
- Review [platform-specific customizations](customization.md) for production deployment
- Compare with [x86-based ECU](../x86-based_ECU/index.md) configurations