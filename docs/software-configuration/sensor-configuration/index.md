# Sensor Configuration

Configure sensors for LSA vehicles with Autoware (platform-agnostic).

## Supported Sensors

- **[LiDAR](../../hardware-configuration/Sensors-and-Actuators/lidars.md)**: 3D environment perception
- **[Cameras](../../hardware-configuration/Sensors-and-Actuators/index.md)**: Object detection and classification
- **[CAN Bus](../../hardware-configuration/Sensors-and-Actuators/index.md)**: Vehicle control interface
- **[GNSS/IMU](../../hardware-configuration/Sensors-and-Actuators/index.md)**: Localization and navigation
- **[Radar](../../hardware-configuration/Sensors-and-Actuators/index.md)**: Additional perception (optional)

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

### Time Synchronization

For the tight real-time communication required for autonomous vehicles, all ECUs and sensors must be
synchronized using Precision Time Protocol (PTP) or other bus or sensor specific synchronization
methods.

Please refer to the 
[PTP Architecture Guide](https://tier4.github.io/sync_tooling/ptp-architecture-guide/), part of the
SYNC.TOOLING diagnostic suite.
This guide provides guidance on how to configure synchronization across machines, recipes for common
setups, command line examples, and Ansible roles.

## LiDAR Configuration

### Velodyne LiDAR

Velodyne LiDARs use UDP packets for data transmission over Ethernet. For hardware specifications and supported models, see [Velodyne Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#velodyne-3d-lidar-sensors).

#### Installation

```bash
# Install Velodyne ROS 2 driver
sudo apt install ros-humble-velodyne
```

#### ARM-Specific Configuration

For ARM platforms like NVIDIA Jetson:

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

Ouster LiDARs provide both point cloud and IMU data. For hardware specifications and supported models, see [Ouster Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#ouster-3d-lidar-sensors).

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

#### ARM-Specific DMA Optimization

For ARM platforms, enable DMA transfer for better performance:

```yaml
# ansible/roles/ouster_arm/tasks/main.yml
---
- name: Configure Ouster with DMA transfer
  template:
    src: ouster_dma_config.j2
    dest: /etc/ros2/ouster_config.yaml
  vars:
    dma_enabled: true
    buffer_size_mb: 256
```

### Hesai LiDAR

Hesai LiDARs are popular in autonomous vehicle applications. For hardware specifications and supported models, see [Hesai Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#hesai-3d-lidar-sensors).

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

### RoboSense LiDAR

RoboSense LiDARs offer various models for different autonomous driving applications. For hardware specifications and supported models, see [RoboSense Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#robosense-3d-lidar-sensors).

#### Installation

```bash
# Clone and build RoboSense driver
cd ~/autoware_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd ~/autoware_ws
colcon build --packages-select rslidar_sdk
```

#### Configuration

```yaml
# ~/autoware_config/sensors/robosense_config.yaml
/**:
  ros__parameters:
    lidar_type: "RS32"  # Options: RS16, RS32, RSBP, RS128, RS80, RSM1, RSHelios
    device_ip: "192.168.1.200"
    msop_port: 6699
    difop_port: 7788
    start_angle: 0
    end_angle: 360
    min_distance: 0.2
    max_distance: 200
    use_lidar_clock: false
```

### Leishen LiDAR

Leishen provides cost-effective LiDAR solutions for autonomous vehicles. For hardware specifications and supported models, see [Leishen Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#leishen-3d-lidar-sensors).

#### Installation

```bash
# Clone and build Leishen driver
cd ~/autoware_ws/src
git clone https://github.com/leishen-lidar/LSLidar_ROS2_driver.git
cd ~/autoware_ws
colcon build --packages-select lslidar_driver
```

#### Configuration

```yaml
# ~/autoware_config/sensors/leishen_config.yaml
/**:
  ros__parameters:
    device_ip: "192.168.1.222"
    device_port: 2368
    device_type: "C32"  # Options: C16, C32, CH32, CH128, C32W
    frame_id: "lslidar"
    scan_frequency: 10.0
    min_range: 0.15
    max_range: 150.0
```

### Livox LiDAR

Livox offers solid-state LiDAR technology with unique non-repetitive scanning patterns. For hardware specifications and supported models, see [Livox Hardware Information](../../hardware-configuration/Sensors-and-Actuators/lidars.md#livox-3d-lidar-sensors).

#### Installation

```bash
# Install Livox ROS 2 driver
sudo apt install ros-humble-livox-ros2-driver
```

#### Configuration

```yaml
# ~/autoware_config/sensors/livox_config.yaml
/**:
  ros__parameters:
    broadcast_code: "3JEDHC900100791"  # Device broadcast code
    enable_lidar_bag: false
    enable_imu_bag: false
    extrinsic_parameter_source: 0  # 0: broadcast, 1: from config file
    enable_high_sensitivity: false
    frame_id: "livox_frame"
```

### Robin W Wide FOV LiDAR

Robin W offers wide field of view LiDAR technology for comprehensive environment sensing. For hardware specifications and supported models, see [Robin W Hardware Information](../../hardware-configuration/Sensors-and-Actuators/index.md#robin-w-wide-fov-lidar).

#### Installation

```bash
# Robin W LiDAR typically uses manufacturer-provided SDK
# Follow the installation guide from Seyond for ROS 2 integration
```

#### Configuration

```yaml
# ~/autoware_config/sensors/robin_w_config.yaml
/**:
  ros__parameters:
    device_ip: "192.168.1.250"
    device_port: 7780
    frame_id: "robin_w"
    horizontal_fov: 120.0  # degrees
    vertical_fov: 70.0     # degrees
    min_range: 0.1
    max_range: 150.0
```

## Camera Configuration

For camera hardware specifications and supported models, see [Camera Hardware Information](../../hardware-configuration/Sensors-and-Actuators/index.md).

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

#### GMSL Configuration on AGX Orin

```bash
# Enable GMSL cameras
sudo modprobe nvgmsl
echo "nvgmsl" | sudo tee -a /etc/modules-load.d/nvgmsl.conf

# Configure camera parameters
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=YUYV
```

#### GMSL Launch Configuration

```yaml
# ~/autoware_config/sensors/gmsl_config.yaml
gmsl_camera:
  ros__parameters:
    camera_info_url: "file:///opt/autoware/calibration/gmsl_camera.yaml"
    video_device: "/dev/video0"
    pixel_format: "YUYV"
    image_width: 1920
    image_height: 1080
    framerate: 30.0
```

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

CAN (Controller Area Network) is essential for vehicle control and telemetry. For CAN interface hardware information, see [CAN Bus Hardware Information](../../hardware-configuration/Sensors-and-Actuators/index.md).

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

#### ARM-Specific CAN Configuration

For ARM platforms, configure CAN with optimized settings:

```bash
# Configure CAN filters for efficiency
sudo ip link set can0 type can bitrate 500000 \
  sample-point 0.875 restart-ms 100
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

For GNSS/IMU hardware specifications and supported models, see [GNSS/IMU Hardware Information](../../hardware-configuration/Sensors-and-Actuators/index.md).

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

### Time Synchronization Issues

PTP is a high-precision protocol and as such can be sensitive to network load, lacking hardware
capabilities, or setup issues. Make sure to follow the
[PTP Architecture Guide](https://tier4.github.io/sync_tooling/ptp-architecture-guide/) for best
practices and troubleshooting steps, and ensure that your setup satisfies the requirements of all
devices in the system.

First steps include checking the sensors' web interfaces or diagnostics and looking for PTP4L or
PHC2SYS errors in the system logs:

```bash
# Replace ptp4l and phc2sys with the service names used in your setup
sudo journalctl -u ptp4l -u phc2sys
```

For advanced setups, the [SYNC.TOOLING](https://tier4.github.io/sync_tooling/) suite provides tools
for real-time monitoring of synchronization status, ROS 2 diagnostics, and a WEB UI for
on-vehicle troubleshooting and offline analysis.

## Performance Considerations

- Use dedicated network interfaces for high-bandwidth sensors
- Configure CPU affinity for sensor driver nodes
- Enable jumbo frames for GigE cameras
- Use hardware time synchronization (PTP) when available
- Monitor system resources with `htop` and `iotop`

## Next Steps

- Proceed to [x86_64-based ECU Configuration](../x86_64-based_ECU/index.md) for Intel/AMD platforms
- Or [ARM-based ECU Configuration](../ARM-based_ECU/index.md) for NVIDIA Jetson platforms
- Configure sensor calibration using Autoware's calibration tools
- Set up sensor fusion parameters for optimal perception performance
