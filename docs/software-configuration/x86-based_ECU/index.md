# Software Configuration for x86-based ECUs

This guide provides comprehensive instructions for deploying Autoware on x86-based ECUs (Intel/AMD platforms) for Low Speed Autonomy vehicles.

## System Preparation

### BIOS/UEFI Configuration

Configure BIOS settings for optimal performance:

```
1. Disable Secure Boot (required for custom kernel modules)
2. Enable Above 4G Decoding (for GPU memory mapping)
3. Set Power Profile to "Performance"
4. Disable C-States for reduced latency
5. Enable Intel VT-x/AMD-V (for containerization)
6. Set PCIe to Gen 4.0 (if supported)
```

### Operating System Installation

Follow the base [Deployment Setup Guide](../deployment-setup/index.md) for Ubuntu 22.04 installation, then apply x86-specific configurations:

```bash
# Install x86-specific packages
sudo apt install -y \
  intel-microcode \
  amd64-microcode \
  linux-tools-generic \
  cpufrequtils \
  i7z \
  powertop

# Install real-time kernel (optional but recommended)
sudo apt install -y linux-realtime
```

## CUDA and GPU Configuration

### 1. CUDA Toolkit Installation

(This part should be moved to hardware-dependent instructions.)

Using the following instructions to install the toolkit of nVidia CUDA. 

```bash
# Add NVIDIA network package repositories
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Install CUDA toolkit
sudo apt install -y cuda-12-3

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda-12.3/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Verify installation
nvidia-smi
nvcc --version
```

### 2. Install NVIDIA Drivers

```bash
# Remove any existing NVIDIA installations
sudo apt remove --purge '^nvidia-.*'
sudo apt autoremove

# Add NVIDIA PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver
ubuntu-drivers devices  # Check recommended version
sudo apt install nvidia-driver-535  # Or latest recommended

# Reboot
sudo reboot
```

### 3. Configure GPU for Compute

```bash
# Set GPU to persistence mode
sudo nvidia-smi -pm 1

# Set application clock speeds to maximum
sudo nvidia-smi -ac 6001,1980  # RTX 4090 example

# Configure GPU for compute mode
sudo nvidia-smi -c EXCLUSIVE_PROCESS

# Verify configuration
nvidia-smi -q -d PERFORMANCE
```

### 4. Install CUDA Toolkit

```bash
# Install CUDA 12.3 (matching driver version)
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run --silent --toolkit

# Install cuDNN
wget https://developer.download.nvidia.com/compute/cudnn/9.0.0/local_installers/cudnn_9.0.0_linux.deb
sudo dpkg -i cudnn_9.0.0_linux.deb

# Install TensorRT
sudo apt install tensorrt
```

## Sensor Configuration

### LiDAR Integration

#### Velodyne LiDAR Setup
```bash
# Install Velodyne ROS 2 driver
sudo apt install ros-humble-velodyne

# Configure network interface
sudo nmcli con add type ethernet \
  con-name lidar-velodyne \
  ifname enp2s0 \
  ip4 192.168.1.100/24

# Create configuration file
mkdir -p ~/autoware_config/sensors
cat > ~/autoware_config/sensors/velodyne.yaml << EOF
/**:
  ros__parameters:
    device_ip: "192.168.1.201"
    port: 2368
    model: "VLP32C"
    rpm: 600
    time_offset: 0.0
    enabled: true
    read_once: false
    read_fast: false
    repeat_delay: 0.0
EOF
```

#### Ouster LiDAR Setup
```bash
# Install Ouster ROS 2 driver
sudo apt install ros-humble-ros2-ouster

# Configure with DMA optimizations
echo 2048 | sudo tee /proc/sys/vm/nr_hugepages
```

### Camera Configuration

#### USB Cameras
```bash
# Install USB camera drivers
sudo apt install ros-humble-usb-cam

# Configure udev rules for consistent naming
echo 'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", SYMLINK+="video_front"' | \
  sudo tee /etc/udev/rules.d/99-camera.rules
sudo udevadm control --reload-rules
```

#### GigE Vision Cameras
```bash
# Install GigE Vision support
sudo apt install ros-humble-camera-aravis

# Optimize network settings
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
```

### CAN Bus Interface

```bash
# Install SocketCAN utilities
sudo apt install can-utils

# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan

# Configure CAN interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Create systemd service for automatic setup
sudo tee /etc/systemd/system/can-setup.service << EOF
[Unit]
Description=Setup CAN interfaces
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/setup-can.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

## Autoware Deployment

### 1. Install Autoware Components

Follow the general installation from [Deployment Setup](../deployment-setup/index.md), then add x86-specific optimizations:

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

#### 1.4 Post-Installation Verification

This section uses a shell script to verify if the Autoware is successfully deployed to the system. 

##### System Check Script

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

##### Verify the deployment
The last step executes the verification script to verify the deployment. 

```
chmod +x verify-installation.sh
./verify-installation.sh
```

#### 1.5 Add x86-specific optimizations:

```bash
# Install performance analysis tools
sudo apt install -y \
  ros-humble-rqt-top \
  ros-humble-rqt-tf-tree \
  htop \
  nvtop \
  linux-tools-common \
  linux-tools-generic

# Install x86-optimized libraries
sudo apt install -y \
  libtbb-dev \
  libopenblas-dev \
  liblapack-dev
```

### 2. Configure Autoware for x86

Create optimized launch configuration:

```bash
cat > ~/autoware_config/x86_optimization.yaml << EOF
/**:
  ros__parameters:
    # CPU optimization
    use_sim_time: false
    num_threads: 16  # Adjust based on CPU cores
    
    # GPU optimization
    gpu_device_id: 0
    use_tensorrt: true
    tensorrt_precision: "FP16"
    
    # Memory optimization
    pointcloud_buffer_size: 100
    image_buffer_size: 30
    
    # Perception settings
    perception:
      lidar:
        detection_method: "centerpoint"
        use_gpu_preprocessing: true
      camera:
        detection_method: "yolox"
        model_type: "yolox-sPlus"
EOF
```

### 3. System Service Configuration

Create systemd service for Autoware:

```bash
sudo tee /etc/systemd/system/autoware.service << EOF
[Unit]
Description=Autoware Universe
After=network.target

[Service]
Type=simple
User=autoware
Environment="ROS_DOMAIN_ID=42"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
ExecStart=/opt/autoware/scripts/start_autoware.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable autoware.service
```

## Performance Optimization

### CPU Optimization

```bash
# Set CPU governor to performance
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance | sudo tee $cpu
done

# Disable CPU frequency scaling
sudo systemctl disable ondemand

# Pin Autoware processes to specific cores
# Add to launch file:
# <node pkg="..." exec="..." cpu_affinity="0-7">
```

### Memory Optimization

```bash
# Configure huge pages
echo 1024 | sudo tee /proc/sys/vm/nr_hugepages

# Disable transparent huge pages
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/enabled

# Set memory swappiness
echo 10 | sudo tee /proc/sys/vm/swappiness
```

### Network Optimization

```bash
# Increase ring buffer sizes
sudo ethtool -G enp1s0 rx 4096 tx 4096

# Enable interrupt coalescing
sudo ethtool -C enp1s0 adaptive-rx on adaptive-tx on

# Configure IRQ affinity
sudo systemctl enable irqbalance
```

## Performance Evaluation

### Monitoring Tools

1. **System Monitoring**
```bash
# CPU and Memory
htop
# GPU
nvtop
nvidia-smi dmon -s pucvmet
# Network
iftop -i enp1s0
```

2. **ROS 2 Performance**
```bash
# Node performance
ros2 run rqt_top rqt_top
# Topic bandwidth
ros2 topic bw /sensing/lidar/concatenated/pointcloud
# Latency measurement
ros2 run performance_test perf_test -c ROS2 -t Array1k
```

### Expected Performance Metrics

| Component | Target FPS | CPU Usage | GPU Usage | Latency |
|-----------|------------|-----------|-----------|---------|
| LiDAR Detection | 10 Hz | 20-30% | 40-60% | <100ms |
| Camera Detection | 15 Hz | 15-25% | 50-70% | <80ms |
| Planning | 10 Hz | 30-40% | N/A | <50ms |
| Control | 50 Hz | 10-15% | N/A | <20ms |

### Profiling and Optimization

```bash
# CPU profiling
perf record -g ros2 launch autoware_launch autoware.launch.xml
perf report

# GPU profiling
nsys profile -o autoware_profile ros2 launch autoware_launch autoware.launch.xml
nsys-ui autoware_profile.nsys-rep

# Memory profiling
valgrind --tool=massif ros2 run perception_node perception_node
ms_print massif.out.*
```

## Troubleshooting

### Common Issues

1. **GPU Memory Errors**
```bash
# Solution: Reduce batch sizes
export TF_FORCE_GPU_ALLOW_GROWTH=true
```

2. **CPU Throttling**
```bash
# Check throttling
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
# Solution: Improve cooling or reduce workload
```

3. **Network Packet Loss**
```bash
# Check for drops
ethtool -S enp1s0 | grep drop
# Solution: Increase buffer sizes
```

## Next Steps

- Configure [RMW Zenoh](../rmw_zenoh/index.md) for improved middleware performance
- Review [ARM-based ECU](../ARM-based_ECU/index.md) documentation for comparison
- Implement custom [sensor drivers](https://autoware.readthedocs.io/en/latest/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-model/) for your specific hardware
