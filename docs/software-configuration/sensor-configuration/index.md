# Sensor Configuration Guide

This guide provides comprehensive instructions for configuring various sensors used in Low Speed Autonomy vehicles with Autoware. The configurations covered here are platform-agnostic and can be applied to both x86 and ARM-based ECUs.

## Overview

Autoware supports a wide range of sensors for autonomous vehicle perception and localization:

- **LiDAR**: Primary sensors for 3D environment perception
- **Cameras**: Visual perception for object detection and classification
- **CAN Bus**: Vehicle interface for control and telemetry
- **GNSS/IMU**: Localization and navigation sensors
- **Radar**: Additional perception capability (optional)

## Network Configuration

### Ethernet-based Sensors

Many modern sensors use Ethernet for high-bandwidth data transmission. Proper network configuration is crucial for reliable sensor operation.

#### Network Interface Setup

```bash
# List available network interfaces
ip link show

# Configure static IP for sensor network
sudo nmcli con add type ethernet \
  con-name sensor-network \
  ifname enp2s0 \
  ip4 192.168.1.100/24

# Activate the connection
sudo nmcli con up sensor-network
```

#### Network Optimization

```bash
# Increase receive buffer size for high-bandwidth sensors
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728

# Make settings persistent
echo "net.core.rmem_max=134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_default=134217728" | sudo tee -a /etc/sysctl.conf
```

## LiDAR Configuration

### Velodyne LiDAR

Velodyne LiDARs use UDP packets for data transmission over Ethernet.

#### Installation

```bash
# Install Velodyne ROS 2 driver
sudo apt install ros-humble-velodyne
```

#### Configuration

Create a configuration file for your Velodyne sensor:

```yaml
# ~/autoware_config/sensors/velodyne_config.yaml
/**:
  ros__parameters:
    device_ip: "192.168.1.201"
    port: 2368
    model: "VLP32C"  # Options: VLP16, VLP32C, VLS128
    rpm: 600
    time_offset: 0.0
    enabled: true
    read_once: false
    read_fast: false
    repeat_delay: 0.0
    frame_id: "velodyne"
```

#### Launch File

```xml
<!-- velodyne.launch.xml -->
<launch>
  <node pkg="velodyne_driver" exec="velodyne_driver_node" name="velodyne_driver">
    <param from="$(find-pkg-share autoware_config)/sensors/velodyne_config.yaml"/>
  </node>
  
  <node pkg="velodyne_pointcloud" exec="velodyne_convert_node" name="velodyne_convert">
    <param name="model" value="VLP32C"/>
    <remap from="velodyne_packets" to="velodyne_driver/velodyne_packets"/>
  </node>
</launch>
```

### Ouster LiDAR

Ouster LiDARs provide both point cloud and IMU data.

#### Installation

```bash
# Install Ouster ROS 2 driver
sudo apt install ros-humble-ros2-ouster
```

#### Configuration

```yaml
# ~/autoware_config/sensors/ouster_config.yaml
ouster_driver:
  ros__parameters:
    sensor_hostname: "192.168.1.202"
    lidar_port: 7502
    imu_port: 7503
    lidar_mode: "1024x10"  # Options: 512x10, 1024x10, 2048x10
    timestamp_mode: "TIME_FROM_ROS_TIME"
```

### Hesai LiDAR

#### Installation

```bash
# Clone and build Hesai driver
cd ~/autoware_ws/src
git clone https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git
cd ~/autoware_ws
colcon build --packages-select hesai_ros_driver
```

#### Configuration

```yaml
# ~/autoware_config/sensors/hesai_config.yaml
/**:
  ros__parameters:
    lidar_type: "PandarXT32"  # Options: Pandar40P, PandarXT32, etc.
    server_ip: "192.168.1.201"
    lidar_recv_port: 2368
    gps_recv_port: 10110
    start_angle: 0
    resolution: 0.2  # Angular resolution in degrees
```

## Camera Configuration

### USB Cameras

#### Installation

```bash
# Install USB camera driver
sudo apt install ros-humble-usb-cam
```

#### Configuration

```yaml
# ~/autoware_config/sensors/usb_cam_config.yaml
/**:
  ros__parameters:
    video_device: "/dev/video0"
    framerate: 30.0
    io_method: "mmap"
    frame_id: "camera"
    pixel_format: "yuyv"
    image_width: 1920
    image_height: 1080
    camera_name: "front_camera"
```

#### Udev Rules for Consistent Naming

Create udev rules to ensure cameras always have the same device names:

```bash
# Create udev rules file
sudo tee /etc/udev/rules.d/99-cameras.rules << EOF
# Front camera
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="085b", SYMLINK+="video_front"

# Rear camera  
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", SYMLINK+="video_rear"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### GMSL Cameras

GMSL (Gigabit Multimedia Serial Link) cameras are commonly used in automotive applications, especially on NVIDIA platforms.

#### GMSL Configuration on NVIDIA Platforms

```bash
# For NVIDIA platforms, GMSL cameras are typically accessed through V4L2
# List available video devices
v4l2-ctl --list-devices

# Query camera capabilities
v4l2-ctl -d /dev/video0 --list-formats-ext
```

#### GStreamer Pipeline for GMSL

```python
# Example GStreamer pipeline for GMSL camera
def create_gmsl_pipeline(sensor_id=0):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink"
    )
```

### GigE Vision Cameras

GigE Vision cameras use Ethernet for data transmission.

#### Installation

```bash
# Install Aravis library for GigE Vision support
sudo apt install ros-humble-camera-aravis
```

#### Network Configuration for GigE

```bash
# Set up jumbo frames for GigE cameras
sudo ip link set dev enp1s0 mtu 9000

# Configure network buffers
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.rmem_default=33554432
```

#### Configuration

```yaml
# ~/autoware_config/sensors/gige_camera_config.yaml
/**:
  ros__parameters:
    guid: "Basler-21995878"  # Camera serial number
    frame_id: "camera"
    exposure_auto: true
    gain_auto: true
    pixel_format: "BayerRG8"
    packet_size: 1500
    packet_delay: 0
```

## CAN Bus Configuration

CAN (Controller Area Network) is essential for vehicle control and telemetry.

### SocketCAN Setup

```bash
# Install CAN utilities
sudo apt install can-utils

# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # Virtual CAN for testing
```

### Physical CAN Interface

```bash
# Configure CAN interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Verify CAN interface is up
ip -details -statistics link show can0
```

### Virtual CAN for Testing

```bash
# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Test with candump
candump vcan0
```

### CAN DBC File Configuration

```yaml
# ~/autoware_config/vehicle/can_config.yaml
/**:
  ros__parameters:
    can_device: "can0"
    dbc_file_path: "/path/to/vehicle.dbc"
    sender_frames:
      - "STEERING_COMMAND"
      - "THROTTLE_COMMAND"
      - "BRAKE_COMMAND"
    receiver_frames:
      - "VEHICLE_STATUS"
      - "WHEEL_SPEEDS"
      - "STEERING_ANGLE"
```

### Systemd Service for CAN

Create a systemd service to automatically configure CAN interfaces:

```bash
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

# Create the setup script
sudo tee /usr/local/bin/setup-can.sh << EOF
#!/bin/bash
# Configure CAN0
ip link set can0 type can bitrate 500000
ip link set up can0

# Configure CAN1 if available
if [ -e /sys/class/net/can1 ]; then
    ip link set can1 type can bitrate 500000
    ip link set up can1
fi
EOF

sudo chmod +x /usr/local/bin/setup-can.sh
sudo systemctl enable can-setup.service
```

## GNSS/IMU Configuration

### GNSS Receivers

#### Serial Port Configuration

```bash
# List serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Create udev rule for persistent naming
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gnss"' | \
  sudo tee /etc/udev/rules.d/99-gnss.rules
```

#### NMEA Configuration

```yaml
# ~/autoware_config/sensors/gnss_config.yaml
/**:
  ros__parameters:
    port: "/dev/gnss"
    baudrate: 115200
    frame_id: "gnss"
    use_gnss_time: false
    publish_rate: 10.0
```

### IMU Configuration

```yaml
# ~/autoware_config/sensors/imu_config.yaml
/**:
  ros__parameters:
    port: "/dev/ttyUSB1"
    baudrate: 921600
    frame_id: "imu"
    imu_rate: 200
    filter_rate: 100
    mag_rate: 100
    enable_magnetometer: false
```

## Sensor Calibration

### Time Synchronization

Accurate time synchronization is crucial for sensor fusion:

```bash
# Install PTP for hardware time synchronization
sudo apt install linuxptp

# Configure PTP
sudo tee /etc/linuxptp/ptp4l.conf << EOF
[global]
priority1 128
priority2 128
domainNumber 0
clockClass 248
clockAccuracy 0xFE
offsetScaledLogVariance 0xFFFF
free_running 0
freq_est_interval 1
EOF

# Start PTP service
sudo systemctl enable ptp4l
sudo systemctl start ptp4l
```

### Sensor Launch Order

Create a master launch file to ensure sensors start in the correct order:

```xml
<!-- sensors.launch.xml -->
<launch>
  <!-- Start time synchronization first -->
  <node pkg="sensor_sync" exec="time_sync_node" name="time_sync"/>
  
  <!-- Start LiDARs -->
  <include file="$(find-pkg-share velodyne_driver)/launch/velodyne.launch.xml"/>
  
  <!-- Start cameras after LiDARs -->
  <include file="$(find-pkg-share usb_cam)/launch/camera.launch.xml">
    <arg name="camera_name" value="front_camera"/>
  </include>
  
  <!-- Start GNSS/IMU -->
  <include file="$(find-pkg-share nmea_navsat_driver)/launch/nmea_navsat.launch.xml"/>
  
  <!-- Start CAN interface last -->
  <node pkg="socketcan_bridge" exec="socketcan_bridge_node" name="can_bridge">
    <param name="can_device" value="can0"/>
  </node>
</launch>
```

## Troubleshooting

### Network Issues

```bash
# Check if sensor is reachable
ping 192.168.1.201

# Monitor network traffic
sudo tcpdump -i enp2s0 -n port 2368

# Check for packet drops
ethtool -S enp2s0 | grep -i drop
```

### USB Issues

```bash
# List USB devices
lsusb -v

# Check USB bandwidth usage
cat /sys/kernel/debug/usb/devices

# Reset USB device
sudo usbreset /dev/bus/usb/001/002
```

### CAN Issues

```bash
# Monitor CAN bus
candump can0

# Check CAN statistics
ip -details -statistics link show can0

# Send test message
cansend can0 123#DEADBEEF
```

## Performance Considerations

- Use dedicated network interfaces for high-bandwidth sensors
- Configure CPU affinity for sensor driver nodes
- Enable jumbo frames for GigE cameras
- Use hardware time synchronization (PTP) when available
- Monitor system resources with `htop` and `iotop`

## Next Steps

- Proceed to [x86-based ECU Configuration](../x86-based_ECU/index.md) for Intel/AMD platforms
- Or [ARM-based ECU Configuration](../ARM-based_ECU/index.md) for NVIDIA Jetson platforms
- Configure sensor calibration using Autoware's calibration tools
- Set up sensor fusion parameters for optimal perception performance