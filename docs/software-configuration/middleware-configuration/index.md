# Middleware Configuration

## Overview

ROS 2 supports multiple middleware implementations (RMW) that handle the underlying communication between nodes. While DDS (Data Distribution Service) is the default middleware, alternative implementations like Zenoh can provide significant advantages for specific use cases, particularly in Low Speed Autonomy applications.

## Available Middleware Options

### 1. DDS Implementations (Default)
- **Fast-RTPS**: Default in ROS 2 Humble
- **CycloneDDS**: Lightweight, high-performance option
- **RTI Connext**: Commercial solution with advanced features

### 2. Zenoh (Recommended for LSA)
A next-generation middleware that has achieved Tier 1 support in ROS 2, offering significant performance and operational benefits for autonomous vehicle deployments.

## Why Consider Alternative Middleware?

The choice of middleware can significantly impact:
- **Performance**: Message latency, throughput, and resource usage
- **Network Compatibility**: WiFi, cellular, and internet connectivity
- **Configuration Complexity**: Setup and tuning requirements
- **Debugging Capability**: Available tools and monitoring options

## Zenoh for Low Speed Autonomy

### Performance Benefits
- **Lower Latency**: Up to 50% reduction in message latency compared to DDS
- **Higher Throughput**: Better performance with large data streams (pointclouds, images)
- **Efficient Resource Usage**: Lower CPU and memory footprint
- **Wireless Optimization**: Superior performance on WiFi and cellular networks

### Operational Advantages
- **Zero Configuration**: Works out-of-the-box without complex QoS tuning
- **Cloud Native**: Seamless operation across local networks and internet
- **Cellular Support**: Native unicast support for 4G/5G networks
- **Simplified Debugging**: Built-in tools for monitoring and troubleshooting

### LSA-Specific Benefits
- **Multi-Vehicle Coordination**: Efficient fleet communication
- **Remote Monitoring**: Native support for cloud connectivity
- **Edge Computing**: Optimized for distributed architectures
- **Dynamic Discovery**: Better handling of intermittent connections

## Installation

### Prerequisites
Ensure you have completed the base Autoware installation from the [Getting Started](../getting-started/index.md) guide.

### Install Zenoh RMW

```bash
# Add the Zenoh repository
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | \
  sudo tee /etc/apt/sources.list.d/zenoh.list > /dev/null
sudo apt update

# Install RMW Zenoh
sudo apt install -y ros-humble-rmw-zenoh-cpp
```

## Configuration

### Basic Setup

1. **Set the RMW Implementation**
```bash
# Add to ~/.bashrc for permanent configuration
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc

# Or set temporarily for current session
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

2. **Configure Zenoh Router (Optional but Recommended)**

Create a Zenoh configuration file:
```bash
cat > ~/zenoh_config.json5 << EOF
{
  mode: "router",
  listen: {
    endpoints: [
      "tcp/0.0.0.0:7447",
      "udp/0.0.0.0:7447"
    ]
  },
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446",
      interface: "auto"
    }
  }
}
EOF
```

3. **Start Zenoh Router**
```bash
# Install Zenoh router
sudo apt install -y zenoh

# Run with configuration
zenohd -c ~/zenoh_config.json5
```

### Advanced Configuration

#### Network-Specific Settings

For different network environments:

```bash
# WiFi Networks
export ZENOH_MULTICAST_SCOUTING=true
export ZENOH_UNICAST_SCOUTING=false

# Cellular Networks (4G/5G)
export ZENOH_MULTICAST_SCOUTING=false
export ZENOH_UNICAST_SCOUTING=true
export ZENOH_PEERS="tcp/router-ip:7447"

# Mixed Networks
export ZENOH_MULTICAST_SCOUTING=true
export ZENOH_UNICAST_SCOUTING=true
```

#### Performance Tuning

```bash
# Optimize for low latency
export ZENOH_CACHE_SIZE=0
export ZENOH_BATCHING_ENABLED=false

# Optimize for high throughput
export ZENOH_CACHE_SIZE=10000
export ZENOH_BATCHING_ENABLED=true
export ZENOH_BATCH_SIZE=65536
```

## Integration with Autoware

### Launch File Configuration

Modify your Autoware launch files to use Zenoh:

```xml
<!-- autoware_zenoh.launch.xml -->
<launch>
  <!-- Set RMW implementation -->
  <set_env name="RMW_IMPLEMENTATION" value="rmw_zenoh_cpp"/>
  
  <!-- Include standard Autoware launch -->
  <include file="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
    <arg name="sensor_model" value="$(var sensor_model)"/>
  </include>
</launch>
```

### Multi-Vehicle Setup

For fleet deployments:

```bash
# Vehicle 1
export ROS_DOMAIN_ID=1
export ZENOH_SESSION_NAME="vehicle_1"

# Vehicle 2
export ROS_DOMAIN_ID=2
export ZENOH_SESSION_NAME="vehicle_2"

# Central Router (Fleet Coordinator)
zenohd --id fleet_router --listen tcp/0.0.0.0:7447
```

## Switching Between Middleware

You can easily switch between different RMW implementations:

```bash
# Use Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Switch back to Fast DDS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Use Cyclone DDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Monitoring and Debugging

### Zenoh Tools

```bash
# Install Zenoh tools
sudo apt install -y zenoh-tools

# Monitor active sessions
z_info

# Scout for Zenoh nodes
z_scout

# Monitor data flow
z_sub -k "/**"
```

### ROS 2 Introspection

```bash
# List active nodes
ros2 node list

# Check middleware in use
ros2 doctor --report | grep middleware

# Monitor topic bandwidth
ros2 topic bw /sensing/lidar/concatenated/pointcloud
```

## Troubleshooting

### Common Issues

1. **Nodes not discovering each other**
   ```bash
   # Check multicast is enabled
   ip link show | grep MULTICAST
   
   # Verify Zenoh router is running
   ps aux | grep zenohd
   ```

2. **High latency on WiFi**
   ```bash
   # Disable power saving
   sudo iw dev wlan0 set power_save off
   
   # Set WiFi to performance mode
   sudo iwconfig wlan0 rate 54M fixed
   ```

3. **Connection issues over cellular**
   ```bash
   # Use explicit peer configuration
   export ZENOH_PEERS="tcp/central-server-ip:7447"
   ```

### Fallback to DDS

If issues persist, you can always fall back to DDS:
```bash
unset RMW_IMPLEMENTATION
# Or explicitly set
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Best Practices

1. **Development**: Use Cyclone DDS for simplicity
2. **Testing**: Validate with target middleware early
3. **Production**: Deploy with Zenoh for optimal performance
4. **Fleet**: Use centralized Zenoh routers
5. **Remote**: Configure explicit peers for cellular

## Next Steps

- Continue to [Sensor Configuration](../sensor-configuration/index.md)
- Review platform-specific guides: [x86 ECUs](../x86-based_ECU/index.md) or [ARM ECUs](../ARM-based_ECU/index.md)
- Learn about [Zenoh architecture](https://zenoh.io/docs/)
