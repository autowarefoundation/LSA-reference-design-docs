# Deployment Setup Guide

This guide provides the foundational setup required for deploying Autoware on both AMD64 and ARM64 platforms running Ubuntu 22.04. Follow these steps before proceeding to platform-specific ECU configurations.

## Requirements on Target Environment

Please assure the requirements for hardware and operating systems are met before continuing to deploy the Autoware.

### Hardware Requirements

#### Minimum Hardware Specifications
- **CPU**: 8 cores (AMD64 or ARM64)
- **RAM**: 16 GB
- **Storage**: 128 GB SSD/NVMe
- **GPU**: NVIDIA GPU with Compute Capability 7.0+ (required for perception)

#### Recommended Specifications
- **CPU**: 16+ cores
- **RAM**: 32 GB or more
- **Storage**: 256 GB NVMe SSD
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

### 2. Configure System Limits for better performance

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

(This part should be moved to hardware-dependent instructions.)

Using the following instructions to install the toolkit of nVidia CUDA. 

### For AMD64 Platforms

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

### For ARM64 Platforms (NVIDIA Jetson)

For Jetson platforms, CUDA comes pre-installed with JetPack. Verify your installation:

```bash
# Check CUDA version
nvcc --version

# Verify Jetson platform
sudo apt install -y python3-pip
pip3 install jetson-stats
sudo jtop
```

If CUDA is not installed, flash your Jetson with the appropriate JetPack version:
- AGX Orin: JetPack 5.1.2 or later
- Xavier Series: JetPack 5.1 or later

## Ansible Setup for Automated Provisioning

[Ansible](https://docs.ansible.com/) is an open source automation tool, which can help to speed up the deployment process. 

### 1. Install Ansible

```bash
# Install Ansible via pip for latest version
python3 -m pip install --user ansible ansible-lint

# Verify installation
ansible --version
```

### 2. Create Ansible Configuration

Create a project directory for Autoware deployment:

```bash
mkdir -p ~/autoware-deployment/ansible
cd ~/autoware-deployment/ansible

# Create ansible.cfg
cat > ansible.cfg << EOF
[defaults]
inventory = ./inventory/hosts.yml
host_key_checking = False
retry_files_enabled = False
stdout_callback = yaml
callback_whitelist = timer,profile_tasks

[privilege_escalation]
become = True
become_method = sudo
become_ask_pass = False
EOF
```

You can also download the [file](assets/ansible.cfg) and move the configuration file to the working directory.

### 3. Create Inventory File

```bash
mkdir -p inventory
cat > inventory/hosts.yml << EOF
all:
  hosts:
    localhost:
      ansible_connection: local
      ansible_python_interpreter: /usr/bin/python3
  vars:
    autoware_version: "2024.12"
    ros_distro: "humble"
    cuda_version: "12.3"
EOF
```

You can also download the [file](assets/hosts.yml) and move the file to the working directory - "ansible/inventory".

### 4. Create Base Provisioning Playbook

```bash
cat > setup-autoware.yml << EOF
---
- name: Setup Autoware Environment
  hosts: localhost
  become: yes
  
  tasks:
    - name: Add ROS 2 GPG key
      apt_key:
        url: https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
        state: present

    - name: Add ROS 2 repository
      apt_repository:
        repo: "deb http://packages.ros.org/ros2/ubuntu {{ ansible_distribution_release }} main"
        state: present

    - name: Update apt cache
      apt:
        update_cache: yes

    - name: Install ROS 2 Humble desktop
      apt:
        name: ros-humble-desktop
        state: present

    - name: Install ROS 2 development tools
      apt:
        name:
          - python3-colcon-common-extensions
          - python3-rosdep
          - python3-vcstool
        state: present

    - name: Initialize rosdep
      command: rosdep init
      args:
        creates: /etc/ros/rosdep/sources.list.d/20-default.list

    - name: Update rosdep
      become: no
      command: rosdep update
EOF
```

You can also download the [file](assets/setup-autoware.yml) and move the set up environment file to the working directory.

## Autoware Deployment via Debian Packages
(This part should be moved to ECU-dependent deployment.)

### 1. Configure Autoware APT Repository

```bash
# Download and install repository configuration
wget https://github.com/autowarefoundation/autoware/releases/latest/download/autoware-apt-config.deb
sudo dpkg -i autoware-apt-config.deb

# Update package lists
sudo apt update
```

### 2. Deploy Autoware Core Packages

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

### 3. Configure Environment

```bash
# Add Autoware setup to bashrc
echo "source /opt/autoware/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 pkg list | grep autoware
```

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
3. **[RMW Zenoh Setup](../rmw_zenoh/index.md)** for alternative middleware configuration

## Additional Resources

- [Autoware Documentation](https://autoware.readthedocs.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA CUDA Installation Guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
- [Ubuntu Real-Time Kernel Setup](https://ubuntu.com/blog/real-time-ubuntu-released)
