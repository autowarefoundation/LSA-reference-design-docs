# ARM-based ECU Customization Guide

This guide provides detailed instructions for customizing Autoware deployments on ARM-based ECUs, with specific focus on NVIDIA AGX Orin and Jetson platforms.

## Platform-Specific Optimizations

### NVIDIA AGX Orin Configuration

#### Power Management
Configure power modes based on deployment requirements:

```bash
# Development mode - Maximum performance
sudo nvpmodel -m 0  # MAXN mode
sudo jetson_clocks

# Production mode - Balanced performance/efficiency
sudo nvpmodel -m 1  # 30W mode
sudo jetson_clocks --restore
```

#### Memory Configuration
Optimize memory allocation for Autoware workloads:

```bash
# Increase GPU memory allocation
echo "gpu_mem_size=8G" | sudo tee /etc/modprobe.d/tegra.conf

# Configure swap for memory-intensive operations
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### Hardware Acceleration Setup

#### Enable DLA (Deep Learning Accelerator)
```yaml
# ansible/roles/agx_orin_dla/tasks/main.yml
---
- name: Enable DLA cores
  lineinfile:
    path: /etc/environment
    line: "{{ '{{ item }}' }}"
  loop:
    - 'CUDA_VISIBLE_DEVICES=0'
    - 'DLA_VISIBLE_DEVICES=0,1'
    - 'TF_ENABLE_TENSORRT_DLA=1'
```

## Sensor Integration

### LiDAR Configuration

#### Velodyne LiDAR on ARM
```yaml
# ansible/roles/velodyne_arm/tasks/main.yml
---
- name: Install Velodyne driver dependencies
  apt:
    name:
      - ros-humble-velodyne
      - ros-humble-velodyne-pointcloud
    state: present

- name: Configure network interface for LiDAR
  nmcli:
    conn_name: lidar0
    ifname: eth1
    type: ethernet
    ip4: 192.168.1.100/24
    state: present
```

#### Ouster LiDAR with DMA Optimization
```yaml
- name: Configure Ouster with DMA transfer
  template:
    src: ouster_dma_config.j2
    dest: /etc/ros2/ouster_config.yaml
  vars:
    dma_enabled: true
    buffer_size_mb: 256
```

### Camera Integration

#### GMSL Cameras (AGX Orin)
```bash
# Enable GMSL cameras
sudo modprobe nvgmsl
echo "nvgmsl" | sudo tee -a /etc/modules-load.d/nvgmsl.conf

# Configure camera parameters
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=YUYV
```

## Network Configuration

### Real-time Communication Setup

#### Configure TSN (Time-Sensitive Networking)
```yaml
# ansible/roles/tsn_config/tasks/main.yml
---
- name: Install TSN utilities
  apt:
    name:
      - linuxptp
      - ethtool
    state: present

- name: Configure PTP for time synchronization
  template:
    src: ptp4l.conf.j2
    dest: /etc/linuxptp/ptp4l.conf

- name: Enable hardware timestamping
  command: "ethtool -T {{ '{{ tsn_interface }}' }}"
```

### CAN Bus Integration

#### SocketCAN Configuration
```bash
# Enable CAN interfaces
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Configure CAN filters for efficiency
sudo ip link set can0 type can bitrate 500000 \
  sample-point 0.875 restart-ms 100
```

## Storage Optimization

### NVMe Configuration for High-Speed Logging

```yaml
# ansible/roles/storage_optimization/tasks/main.yml
---
- name: Configure NVMe for optimal performance
  lineinfile:
    path: /etc/fstab
    line: '/dev/nvme0n1p1 /var/log/autoware ext4 noatime,nodiratime,nobarrier 0 2'

- name: Set up log rotation for Autoware
  template:
    src: autoware_logrotate.j2
    dest: /etc/logrotate.d/autoware
```

### SD Card Optimization (Jetson Nano/Xavier NX)
```bash
# Reduce SD card wear
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
echo "vm.vfs_cache_pressure=50" | sudo tee -a /etc/sysctl.conf

# Move temporary files to RAM
echo "tmpfs /tmp tmpfs defaults,noatime,mode=1777 0 0" | sudo tee -a /etc/fstab
```

## Performance Tuning

### CPU Governor Settings
```bash
# Set performance governor for all cores
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
  echo performance | sudo tee $cpu
done

# Make persistent
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

### GPU Optimization
```yaml
# ansible/roles/gpu_optimization/tasks/main.yml
---
- name: Set GPU clock to maximum
  command: nvidia-smi -pm 1

- name: Configure GPU memory growth
  lineinfile:
    path: /etc/environment
    regexp: '^TF_FORCE_GPU_ALLOW_GROWTH='
    line: 'TF_FORCE_GPU_ALLOW_GROWTH=true'

- name: Set CUDA device order
  lineinfile:
    path: /etc/environment
    regexp: '^CUDA_DEVICE_ORDER='
    line: 'CUDA_DEVICE_ORDER=PCI_BUS_ID'
```

## Custom Kernel Modules

### Building Custom Drivers
```makefile
# Makefile for custom ARM kernel module
obj-m += autoware_custom_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
```

### Loading Modules at Boot
```bash
# Add to /etc/modules-load.d/autoware.conf
autoware_custom_driver
can_isotp
peak_usb
```

## Security Hardening

### Secure Boot Configuration
```yaml
# ansible/roles/secure_boot/tasks/main.yml
---
- name: Enable secure boot
  command: mokutil --enable-validation

- name: Sign custom kernel modules
  command: |
    /usr/src/linux-headers-$(uname -r)/scripts/sign-file \
    sha256 /var/lib/shim-signed/mok/MOK.priv \
    /var/lib/shim-signed/mok/MOK.der \
    {{ '{{ module_path }}' }}
```

### AppArmor Profiles
```bash
# Create AppArmor profile for Autoware
sudo aa-genprof /opt/autoware/bin/autoware_launch

# Example profile snippet
/opt/autoware/bin/autoware_launch {
  #include <abstractions/base>
  
  capability sys_nice,
  capability sys_resource,
  
  /opt/autoware/** r,
  /var/log/autoware/** rw,
  /dev/nvidia* rw,
}
```

## Monitoring and Diagnostics

### System Health Monitoring
```yaml
# ansible/roles/monitoring/tasks/main.yml
---
- name: Install monitoring tools
  apt:
    name:
      - nvidia-jetson-stats
      - htop
      - iotop
      - nethogs
    state: present

- name: Configure Prometheus node exporter
  template:
    src: node_exporter.service.j2
    dest: /etc/systemd/system/node_exporter.service
```

### Performance Profiling
```bash
# Profile Autoware with Nsight Systems
nsys profile -t cuda,nvtx,osrt,cudnn,cublas \
  -o autoware_profile \
  ros2 launch autoware_launch autoware.launch.xml

# Analyze with Nsight Compute
ncu --target-processes all \
  --metrics gpu__time_duration.sum \
  ros2 run perception_node perception_node
```

## Deployment Automation

### Ansible Playbook for Complete Setup

(There are errors on rendering the content in this part.)

```yaml
# deploy_arm_ecu.yml
---
- hosts: arm_ecus
  become: yes
  vars:
    ecu_type: "agx_orin"
    autoware_version: "2024.12"
  
  roles:
    - role: base_system
    - role: nvidia_drivers
    - role: autoware_core
    - role: sensor_drivers
    - role: network_config
    - role: performance_tuning
    - role: monitoring

  post_tasks:
    - name: Verify deployment
      command: ros2 doctor
      register: ros2_doctor_output

    - name: Display status
      debug:
       msg: "Deployment complete. ROS 2 status:"
```

## Summary

Customizing ARM-based ECUs for Autoware requires careful attention to:

1. **Hardware-specific optimizations** - Leveraging NVIDIA accelerators
2. **Sensor integration** - Proper driver configuration and DMA setup
3. **Network configuration** - Real-time communication requirements
4. **Performance tuning** - CPU, GPU, and memory optimization
5. **Security hardening** - Production-ready deployments

These customizations ensure optimal performance and reliability for autonomous vehicle applications on ARM platforms.

