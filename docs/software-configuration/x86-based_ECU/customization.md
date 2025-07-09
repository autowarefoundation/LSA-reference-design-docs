# x86-based ECU Customization Guide

This guide provides advanced customization options for x86-based ECUs running Autoware, focusing on platform-specific optimizations and configurations.

## Hardware-Specific Optimizations

### Intel Platform Optimizations

#### Intel Performance Libraries
```bash
# Install Intel MKL for optimized math operations
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo add-apt-repository "deb https://apt.repos.intel.com/oneapi all main"
sudo apt update
sudo apt install intel-oneapi-mkl-devel

# Configure environment
echo 'source /opt/intel/oneapi/setvars.sh' >> ~/.bashrc
source ~/.bashrc
```

#### AVX-512 Optimization
```yaml
# ansible/roles/intel_optimization/tasks/main.yml
---
- name: Enable AVX-512 optimizations
  lineinfile:
    path: /etc/environment
    line: "{{ '{{ item }}' }}"
  loop:
    - 'CXXFLAGS="-march=native -O3 -mavx512f"'
    - 'OMP_NUM_THREADS={{ '{{ ansible_processor_vcpus }}' }}'
    - 'MKL_NUM_THREADS={{ '{{ ansible_processor_vcpus }}' }}'

- name: Install Intel VTune profiler
  apt:
    name: intel-oneapi-vtune
    state: present
```

### AMD Platform Optimizations

#### AMD BLIS/FLAME Libraries
```bash
# Install AMD optimized libraries
sudo apt install libblis-dev libflame-dev

# Configure for Zen architecture
export BLIS_ARCH_TYPE=zen
export BLIS_NUM_THREADS=$(nproc)
```

#### ROCm Integration (AMD GPUs)
```bash
# Add ROCm repository
wget https://repo.radeon.com/rocm/rocm.gpg.key
sudo apt-key add rocm.gpg.key
echo 'deb [arch=amd64] https://repo.radeon.com/rocm/apt/5.7 ubuntu main' | \
  sudo tee /etc/apt/sources.list.d/rocm.list
sudo apt update

# Install ROCm
sudo apt install rocm-dev rocm-libs hipcub
```

## Multi-GPU Configuration

### NVIDIA Multi-GPU Setup

```yaml
# ansible/roles/multi_gpu/tasks/main.yml
---
- name: Configure NVIDIA Multi-GPU
  template:
    src: multi_gpu_config.j2
    dest: /etc/autoware/multi_gpu.yaml
  vars:
    gpu_assignments:
      perception_lidar: 0
      perception_camera: 1
      planning: 0
      visualization: 1

- name: Set CUDA visible devices for services
  lineinfile:
    path: "/etc/systemd/system/autoware-{{ '{{ item.service }}' }}.service"
    regexp: '^Environment="CUDA_VISIBLE_DEVICES'
    line: "Environment=\"CUDA_VISIBLE_DEVICES={{ '{{ item.gpu }}' }}\""
  loop:
    - { service: 'perception', gpu: '0' }
    - { service: 'planning', gpu: '1' }
```

### GPU Memory Management
```bash
# Create GPU memory allocation script
cat > /usr/local/bin/configure_gpu_memory.sh << 'EOF'
#!/bin/bash
# Configure GPU memory allocation

# Enable unified memory
nvidia-smi -c 3

# Set memory growth for TensorFlow
export TF_FORCE_GPU_ALLOW_GROWTH=true

# Limit GPU memory per process
export CUDA_VISIBLE_MEMORY=8192  # MB

# Configure PyTorch
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512
EOF

chmod +x /usr/local/bin/configure_gpu_memory.sh
```

## Advanced Networking

### SR-IOV Configuration

```bash
# Enable SR-IOV on Intel NICs
echo 7 | sudo tee /sys/class/net/enp1s0f0/device/sriov_numvfs

# Create virtual functions
sudo ip link set enp1s0f0 vf 0 mac 00:11:22:33:44:55
sudo ip link set enp1s0f0 vf 0 vlan 100
```

### DPDK Integration

```yaml
# ansible/roles/dpdk_setup/tasks/main.yml
---
- name: Install DPDK
  apt:
    name:
      - dpdk
      - dpdk-dev
      - libdpdk-dev
    state: present

- name: Configure huge pages for DPDK
  sysctl:
    name: vm.nr_hugepages
    value: 2048
    state: present

- name: Bind network interface to DPDK
  command: "dpdk-devbind.py --bind=vfio-pci {{ '{{ dpdk_interface }}' }}"
```

## Storage Optimization

### NVMe RAID Configuration

```bash
# Create software RAID 0 for maximum performance
sudo mdadm --create /dev/md0 --level=0 --raid-devices=2 /dev/nvme0n1 /dev/nvme1n1

# Format with optimized settings
sudo mkfs.ext4 -E stride=128,stripe-width=256 /dev/md0

# Mount with performance options
echo '/dev/md0 /data ext4 noatime,nodiratime,nobarrier 0 0' | sudo tee -a /etc/fstab
```

### RAM Disk for Temporary Data

```bash
# Create RAM disk for high-frequency data
sudo mkdir -p /mnt/ramdisk
echo 'tmpfs /mnt/ramdisk tmpfs size=8G,mode=1777 0 0' | sudo tee -a /etc/fstab
sudo mount /mnt/ramdisk

# Configure ROS 2 to use RAM disk
export ROS_HOME=/mnt/ramdisk/ros
export ROS_LOG_DIR=/mnt/ramdisk/ros/log
```

## Custom Kernel Configuration

### Real-Time Kernel Patches

```bash
# Download and compile RT kernel
cd /usr/src
sudo wget https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.15.139.tar.xz
sudo wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.15/patch-5.15.139-rt74.patch.xz

sudo tar xf linux-5.15.139.tar.xz
cd linux-5.15.139
sudo xzcat ../patch-5.15.139-rt74.patch.xz | sudo patch -p1

# Configure for real-time
sudo make menuconfig
# Enable: CONFIG_PREEMPT_RT, CONFIG_HIGH_RES_TIMERS, CONFIG_NO_HZ_FULL
```

### Custom Driver Development

```c
// custom_sensor_driver.c
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

static int sensor_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    // Enable DMA
    if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
        if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
            dev_err(&pdev->dev, "No suitable DMA mask available\n");
            return -ENODEV;
        }
    }
    
    // Enable MSI-X interrupts
    int nvec = pci_alloc_irq_vectors(pdev, 1, 8, PCI_IRQ_MSIX);
    if (nvec < 0)
        return nvec;
    
    return 0;
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Custom Autoware Sensor Driver");
```

## Security Hardening

### SELinux Policies

```bash
# Create custom SELinux policy for Autoware
cat > autoware.te << 'EOF'
policy_module(autoware, 1.0.0)

require {
    type autoware_t;
    type autoware_exec_t;
    class capability { sys_nice sys_resource };
}

# Allow Autoware to set real-time priorities
allow autoware_t self:capability { sys_nice sys_resource };

# Allow GPU access
allow autoware_t gpu_device_t:chr_file rw_file_perms;
EOF

# Compile and install policy
checkmodule -M -m -o autoware.mod autoware.te
semodule_package -o autoware.pp -m autoware.mod
sudo semodule -i autoware.pp
```

### Network Security

```yaml
# ansible/roles/network_security/tasks/main.yml
---
- name: Configure firewall for Autoware
  ufw:
    rule: allow
    port: "{{ '{{ item }}' }}"
    proto: tcp
  loop:
    - 11311  # ROS Master (if using ROS 1 bridge)
    - 11611-11711  # DDS discovery
    - 7400-7500  # DDS communication

- name: Enable network namespaces
  sysctl:
    name: net.ipv4.ip_forward
    value: 1
    state: present
```

## Performance Monitoring

### Custom Monitoring Stack

```yaml
# docker-compose.monitoring.yml
version: '3.8'
services:
  prometheus:
    image: prom/prometheus
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
    ports:
      - "9090:9090"
  
  grafana:
    image: grafana/grafana
    ports:
      - "3000:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=autoware
  
  node-exporter:
    image: prom/node-exporter
    pid: host
    volumes:
      - /proc:/host/proc:ro
      - /sys:/host/sys:ro
```

### Custom Metrics Collection

```python
#!/usr/bin/env python3
# autoware_metrics_exporter.py

import rclpy
from rclpy.node import Node
from prometheus_client import Counter, Histogram, Gauge, start_http_server

class AutowareMetricsExporter(Node):
    def __init__(self):
        super().__init__('autoware_metrics_exporter')
        
        # Define Prometheus metrics
        self.pointcloud_latency = Histogram(
            'autoware_pointcloud_latency_seconds',
            'Pointcloud processing latency'
        )
        self.detection_count = Counter(
            'autoware_detections_total',
            'Total number of object detections'
        )
        self.cpu_usage = Gauge(
            'autoware_cpu_usage_percent',
            'CPU usage percentage'
        )
        
        # Start Prometheus HTTP server
        start_http_server(8000)
        
        # Subscribe to Autoware topics
        self.create_subscription(
            PointCloud2,
            '/perception/lidar/pointcloud',
            self.pointcloud_callback,
            10
        )
```

## Deployment Automation

### Complete Deployment Playbook

```yaml
# deploy_x86_autoware.yml
---
- hosts: x86_ecus
  become: yes
  vars:
    platform_type: "{{ '{% if \"Intel\" in ansible_processor[1] %}intel{% elif \"AMD\" in ansible_processor[1] %}amd{% else %}unknown{% endif %}' }}"
    gpu_count: "{{ '{{ ansible_facts.gpus | length | default(1) }}' }}"
    
  tasks:
    - name: Detect platform and apply optimizations
      include_role:
        name: "{{ '{{ platform_type }}' }}_optimization"
      when: platform_type in ['intel', 'amd']
    
    - name: Configure multi-GPU if available
      include_role:
        name: multi_gpu
      when: gpu_count | int > 1
    
    - name: Apply security hardening
      include_role:
        name: security_hardening
    
    - name: Setup monitoring
      include_role:
        name: monitoring_stack
    
    - name: Validate deployment
      command: ros2 doctor --report
      register: validation_result
    
    - name: Display deployment summary
      debug:
        msg: |
          Deployment completed successfully!
          Platform detected and optimizations applied.
          GPU configuration completed.
          Validation performed with ros2 doctor.
```

## Summary

x86 platform customization for Autoware involves:

1. **Platform-specific optimizations** for Intel/AMD architectures
2. **Multi-GPU configurations** for enhanced perception performance  
3. **Advanced networking** with SR-IOV and DPDK
4. **Storage optimization** using NVMe RAID and RAM disks
5. **Security hardening** with SELinux and network policies
6. **Comprehensive monitoring** for production deployments

These customizations ensure optimal performance and reliability for autonomous vehicle applications on x86 platforms.