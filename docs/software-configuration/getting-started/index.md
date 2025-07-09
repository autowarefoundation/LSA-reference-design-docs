# Getting Started

This guide provides the foundational setup required for deploying Autoware on both AMD64 and ARM64 platforms running Ubuntu 22.04. Follow these steps before proceeding to platform-specific ECU configurations.

## Requirements on Target Environment

Please assure the requirements for hardware and operating systems are met before continuing to deploy the Autoware.

### Hardware Requirements

#### Minimum Specifications
- **CPU**: 8-core x86_64 (Intel/AMD) or ARMv8 (ARM64)
- **RAM**: 32 GB (16 GB for simulation-only)
- **Storage**: 30 GB of free space
- **GPU**: NVIDIA GPU with CUDA 11.8+ and Compute Capability 5.0+ (required for perception)

#### Recommended Specifications
- **CPU**: 16+ cores
- **RAM**: 64 GB or more
- **Storage**: 100 GB+ of free space on SSD
- **GPU**: NVIDIA RTX 3080+ (x86) or AGX Orin (ARM)

### Network Requirements
- Gigabit Ethernet for sensor connectivity
- Internet access for initial setup and package installation
- **Optional**: Secondary network interface for vehicle communication

### Operating System
- **Ubuntu 22.04 LTS (Jammy Jellyfish)**
- Kernel version 5.15 or later
- Real-time kernel is recommended for production deployments

## Preparation for system environments and tools

Given meeting the requirements for hardware and operating systems, the following instructions prepare the system environments and tools for the deployment. 

### 1. Update Base System

Using the following instructions to update the base system of the package installation. The comments started with the symbol '#' and should be ignored while executing the commands. One may copy and paste the command to the terminal.

```bash
# Update package lists and upgrade system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
  build-essential \
  cmake \
  git \
  wget \
  curl \
  gnupg \
  lsb-release \
  software-properties-common \
  python3-pip \
  python3-venv
```

### 2. Configure System Limits for Better Performance

Using the following instructions to optimize system settings for real-time performance: 

```bash
# Add to /etc/security/limits.conf
echo "@autoware - rtprio 98" | sudo tee -a /etc/security/limits.conf
echo "@autoware - memlock unlimited" | sudo tee -a /etc/security/limits.conf

# Configure kernel parameters
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_max=134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=134217728" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## CUDA Toolkit Installation

CUDA installation is hardware-specific and varies significantly between x86 and ARM platforms. Please refer to the appropriate platform-specific guide for detailed CUDA installation instructions:

- **For x86-based ECUs (AMD64)**: See [x86 CUDA Installation Guide](../x86-based_ECU/index.md#cuda-and-gpu-configuration)
- **For ARM-based ECUs (NVIDIA Jetson)**: See [ARM CUDA Installation Guide](../ARM-based_ECU/index.md#cuda-toolkit-installation)

**Important Notes:**
- x86 platforms require manual CUDA toolkit installation from NVIDIA repositories
- ARM Jetson platforms come with CUDA pre-installed through JetPack SDK
- Ensure CUDA version compatibility with your GPU and driver version

## Environment Setup

The Autoware project provides an automated setup script that installs all necessary dependencies and configures your development environment.

### 1. Clone Autoware Repository

```bash
# Clone the Autoware repository
git clone https://github.com/autowarefoundation/autoware.git
cd autoware

# Checkout the 2025.02 branch
git checkout 2025.02
```

### 2. Run Setup Script

The setup script will automatically install ROS 2, development tools, and all required dependencies:

```bash
# Run the setup script
./setup-dev-env.sh
```

This script will:
- Install ROS 2 Humble (if not already installed)
- Install development tools (colcon, vcstool, rosdep)
- Configure your shell environment
- Install additional Autoware dependencies
- Set up pre-commit hooks for development

**Note**: The script may take 10-30 minutes to complete depending on your internet connection and system performance.

### 3. Source the Environment

After the setup completes, source your ROS 2 environment:

```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# For development, you may want to add this to your .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

For more detailed information about the setup process, refer to the [Autoware source installation guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

**Note**: For advanced users who want to customize the installation process using Ansible, please refer to the platform-specific customization guides:
- [x86 Customization Guide](../x86-based_ECU/customization.md)
- [ARM Customization Guide](../ARM-based_ECU/customization.md)

## Autoware Installation via Debian Packages

The NEWSLab team provides pre-built Debian packages for Autoware that simplify the installation process. This method is faster than building from source and ensures consistent deployments.

### 1. Configure Autoware Local Repository

Download and install the appropriate repository configuration package:

#### For x86\_64 (AMD64) Systems:
```bash
# Download the repository configuration package
wget https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_amd64.deb

# Install the repository configuration
sudo dpkg -i autoware-localrepo_2025.2-1_amd64.deb
```

#### For ARM64 Systems (Non-Jetson):
```bash
# Download the repository configuration package  
wget https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_arm64.deb

# Install the repository configuration
sudo dpkg -i autoware-localrepo_2025.2-1_arm64.deb
```

#### For NVIDIA Jetson Platforms (AGX Orin with JetPack 6.0):
```bash
# Download the Jetson-specific repository configuration package
wget https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_jetpack6.0.deb

# Install the repository configuration
sudo dpkg -i autoware-localrepo_2025.2-1_jetpack6.0.deb
```

### 2. Update Package Lists

After installing the repository configuration:

```bash
# Update apt package lists to include the new repository
sudo apt update
```

### 3. Install Autoware Packages

Install the full Autoware stack or specific components:

```bash
sudo apt install -y autoware-full
```

### 4. Configure Environment

Set up your shell environment to use the installed Autoware:

```bash
# Add Autoware setup to bashrc
echo "source /opt/autoware/autoware-env" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 pkg list | grep autoware
```

**Note**: The Debian packages install Autoware to `/opt/autoware` and are designed to coexist with other ROS 2 installations.

## Post-Installation Verification

This section uses a shell script to verify if the Autoware is successfully deployed to the system. 

### 1. System Check Script

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
if [ -f /opt/autoware/autoware-env ]; then
    source /opt/autoware/autoware-env
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

### 2. Test Basic Functionality

```bash
# Test ROS 2 communication
ros2 doctor

# Launch minimal Autoware nodes
ros2 launch autoware_launch logging_simulator.launch.xml
```

## Common Troubleshooting

### CUDA Issues

**Problem**: `nvidia-smi` command not found
```bash
# Solution: Reinstall NVIDIA drivers
sudo apt install --reinstall nvidia-driver-535
sudo reboot
```

**Problem**: CUDA version mismatch
```bash
# Solution: Check compatibility
nvidia-smi  # Check driver version
# Ensure CUDA toolkit matches driver version
```

### ROS 2 Issues

**Problem**: Package not found errors
```bash
# Solution: Update rosdep and install dependencies
cd ~/autoware
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
```

**Problem**: Transform (TF) errors
```bash
# Solution: Check system time synchronization
timedatectl status
sudo apt install ntp
sudo systemctl enable ntp
```

### Network Issues

**Problem**: Cannot download packages
```bash
# Solution: Configure proxy if behind firewall
export http_proxy=http://your-proxy:port
export https_proxy=http://your-proxy:port
sudo -E apt update
```

## Performance Optimization

### CPU Governor
```bash
# Set to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### Memory Settings
```bash
# Disable swap for real-time performance
sudo swapoff -a
# Comment out swap line in /etc/fstab
```

### Network Optimization
```bash
# Increase network buffer sizes
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.wmem_max=134217728
sudo sysctl -w net.ipv4.tcp_rmem="4096 87380 134217728"
sudo sysctl -w net.ipv4.tcp_wmem="4096 65536 134217728"
```

## Next Steps

With the base system configured, proceed to:

1. **[x86-based ECU Configuration](../x86-based_ECU/index.md)** for Intel/AMD platforms
2. **[ARM-based ECU Configuration](../ARM-based_ECU/index.md)** for NVIDIA Jetson platforms
3. **[RMW Zenoh Setup](../middleware-configuration/index.md)** for alternative middleware configuration

## Additional Resources

- [Autoware Documentation](https://autoware.readthedocs.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA CUDA Installation Guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
- [Ubuntu Real-Time Kernel Setup](https://ubuntu.com/blog/real-time-ubuntu-released)
