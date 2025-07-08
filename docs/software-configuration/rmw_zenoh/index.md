# RMW Zenoh Configuration for Autoware

## Overview

RMW Zenoh is a next-generation middleware for ROS 2 that provides significant advantages over traditional DDS implementations. As of ROS 2 Kilted (and backported to Humble), Zenoh has achieved Tier 1 support status, making it a production-ready alternative for Autoware deployments in Low Speed Autonomy applications.

## Why Choose RMW Zenoh for LSA?

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
- **Multi-Vehicle Coordination**: Easy vehicle-to-vehicle communication
- **Remote Monitoring**: Direct cloud connectivity for fleet management
- **Edge Computing**: Efficient offloading to edge servers
- **Scalability**: Handles growing fleet sizes without reconfiguration

## Installation

### Prerequisites

Ensure you have completed the base [Deployment Setup](../deployment-setup/index.md) before proceeding.

### Install RMW Zenoh

#### Method 1: Debian Package (Recommended)

```bash
# Add RMW Zenoh repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:rmw-zenoh/stable
sudo apt update

# Install RMW Zenoh
sudo apt install ros-humble-rmw-zenoh-cpp

# Install Zenoh router (optional but recommended)
sudo apt install zenoh-router
```

#### Method 2: Build from Source

```bash
# Clone repository
cd ~/ros2_ws/src
git clone https://github.com/ros2/rmw_zenoh.git -b humble
git clone https://github.com/eclipse-zenoh/zenoh-c.git
git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-up-to rmw_zenoh_cpp
```

## Configuration

### Basic Configuration

#### 1. Set RMW Implementation

```bash
# Add to ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc

# Verify
echo $RMW_IMPLEMENTATION
```

#### 2. Create Zenoh Configuration

```bash
# Create config directory
mkdir -p ~/.config/autoware/zenoh

# Create basic configuration
cat > ~/.config/autoware/zenoh/config.json5 << 'EOF'
{
  // Zenoh configuration for Autoware LSA
  mode: "client",
  
  // Network configuration
  listen: {
    endpoints: [
      "tcp/0.0.0.0:7447",
      "udp/0.0.0.0:7447"
    ]
  },
  
  // Connect to local router
  connect: {
    endpoints: [
      "tcp/localhost:7447"
    ]
  },
  
  // Scouting for peer discovery
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446",
      interface: "auto"
    },
    delay: 200
  },
  
  // QoS settings optimized for Autoware
  qos: {
    reliability: "reliable",
    congestion_control: "drop",
    express: true
  }
}
EOF
```

### Advanced Configuration

#### Multi-Vehicle Setup

```json5
// zenoh-multi-vehicle.json5
{
  mode: "peer",
  
  // Vehicle identification
  id: "vehicle_001",
  
  // Multi-vehicle discovery
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446"
    },
    gossip: {
      enabled: true,
      multihop: true
    }
  },
  
  // Namespace isolation
  plugins: {
    ros2: {
      namespace: "/vehicle_001",
      domain: 42
    }
  }
}
```

#### Cloud Connectivity

```json5
// zenoh-cloud.json5
{
  mode: "client",
  
  // Connect to cloud router
  connect: {
    endpoints: [
      "tls/cloud.example.com:7447"
    ],
    // Authentication
    auth: {
      usrpwd: {
        user: "vehicle_001",
        password: "${ZENOH_PASSWORD}"
      }
    }
  },
  
  // TLS configuration
  tls: {
    root_ca_certificate: "/etc/autoware/certs/ca.pem",
    client_certificate: "/etc/autoware/certs/client.pem",
    client_private_key: "/etc/autoware/certs/client.key"
  }
}
```

## Deployment Patterns

### 1. Single Vehicle Configuration

```bash
# Start Zenoh router (optional)
zenoh-router --config ~/.config/autoware/zenoh/config.json5 &

# Launch Autoware with Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG=~/.config/autoware/zenoh/config.json5
ros2 launch autoware_launch autoware.launch.xml
```

### 2. Fleet Configuration

```yaml
# ansible/roles/zenoh_fleet/tasks/main.yml
---
- name: Deploy Zenoh router configuration
  template:
    src: zenoh-router.json5.j2
    dest: /etc/zenoh/router.json5
  vars:
    fleet_id: "{{ fleet_name }}"
    vehicle_id: "{{ ansible_hostname }}"

- name: Create systemd service for Zenoh router
  template:
    src: zenoh-router.service.j2
    dest: /etc/systemd/system/zenoh-router.service

- name: Start Zenoh router
  systemd:
    name: zenoh-router
    enabled: yes
    state: started
```

### 3. Edge Computing Integration

```python
#!/usr/bin/env python3
# edge_compute_bridge.py

import zenoh
import json
from autoware_auto_msgs.msg import DetectedObjects

class EdgeComputeBridge:
    def __init__(self):
        # Initialize Zenoh
        self.session = zenoh.open(zenoh.Config.from_file("edge-config.json5"))
        
        # Subscribe to vehicle data
        self.sub = self.session.declare_subscriber(
            "autoware/*/perception/objects",
            self.on_detection
        )
        
        # Publisher for processed results
        self.pub = self.session.declare_publisher(
            "edge/processed/tracks"
        )
    
    def on_detection(self, sample):
        # Process detection data on edge
        objects = json.loads(sample.payload.decode())
        processed = self.process_detections(objects)
        
        # Publish back to vehicles
        self.pub.put(json.dumps(processed))
```

## Performance Tuning

### Network Optimization

```bash
# Increase UDP buffer sizes
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_max=26214400
sudo sysctl -w net.core.wmem_default=26214400

# Make persistent
echo "net.core.rmem_max=26214400" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max=26214400" | sudo tee -a /etc/sysctl.conf
```

### Zenoh-Specific Tuning

```json5
// zenoh-performance.json5
{
  // Performance optimizations
  transport: {
    unicast: {
      max_sessions: 1000,
      max_links: 4
    },
    multicast: {
      max_sessions: 1000
    },
    qos: {
      enabled: true
    },
    link: {
      tx: {
        sequence_number_resolution: 268435456,  // 2^28
        lease: 10000,
        keep_alive: 4,
        batch_size: 65535
      },
      rx: {
        buffer_size: 65535,
        max_message_size: 1073741824  // 1GB
      }
    }
  },
  
  // Shared memory for local communication
  shared_memory: {
    enabled: true,
    size: 268435456  // 256MB
  }
}
```

### Monitoring and Metrics

```bash
# Enable Zenoh REST API
zenoh-router --rest-http-port 8000 &

# Query router statistics
curl http://localhost:8000/@/router/local

# Monitor real-time statistics
watch -n 1 'curl -s http://localhost:8000/@/router/local/stats'
```

## Integration with Autoware

### Launch File Configuration

```xml
<!-- autoware_zenoh.launch.xml -->
<launch>
  <!-- Set RMW implementation -->
  <env name="RMW_IMPLEMENTATION" value="rmw_zenoh_cpp"/>
  
  <!-- Zenoh configuration file -->
  <env name="ZENOH_CONFIG" value="$(find-pkg-share autoware_launch)/config/zenoh.json5"/>
  
  <!-- Vehicle namespace for multi-vehicle -->
  <arg name="vehicle_id" default="vehicle_1"/>
  <group>
    <push-ros-namespace namespace="$(var vehicle_id)"/>
    
    <!-- Include standard Autoware launch -->
    <include file="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml">
      <arg name="use_sim_time" value="false"/>
    </include>
  </group>
</launch>
```

### ROS 2 Parameter Bridge

For hybrid DDS/Zenoh deployments:

```yaml
# zenoh_dds_bridge.yaml
ros2_to_zenoh_bridge:
  ros__parameters:
    # Topics to bridge from ROS 2 (DDS) to Zenoh
    dds_to_zenoh:
      - topic: /sensing/lidar/concatenated/pointcloud
        qos: sensor_data
      - topic: /perception/object_recognition/objects
        qos: reliable
    
    # Topics to bridge from Zenoh to ROS 2 (DDS)
    zenoh_to_dds:
      - topic: /planning/scenario_planning/trajectory
        qos: reliable
      - topic: /control/command/control_cmd
        qos: reliable
```

## Troubleshooting

### Common Issues

1. **Discovery Problems**
```bash
# Check multicast is enabled
ip maddr show | grep 224.0.0.224

# Test Zenoh discovery
zenoh-scouter
```

2. **Performance Issues**
```bash
# Check for packet drops
netstat -su | grep -i drop

# Monitor Zenoh internals
RUST_LOG=debug zenoh-router
```

3. **Connection Failures**
```bash
# Verify firewall rules
sudo ufw status
sudo ufw allow 7447/tcp
sudo ufw allow 7447/udp
sudo ufw allow 7446/udp  # Multicast discovery
```

### Migration from DDS

#### Gradual Migration Strategy

1. **Phase 1**: Run Zenoh in parallel with DDS
```bash
# Keep DDS for critical topics
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run critical_safety_node &

# Use Zenoh for perception/planning
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch perception_pipeline
```

2. **Phase 2**: Use bridge for transition
```bash
ros2 run zenoh_dds_bridge zenoh_dds_bridge --config bridge.yaml
```

3. **Phase 3**: Full Zenoh deployment
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch autoware_launch autoware.launch.xml
```

## Best Practices

### Security
- Always use TLS for cloud connections
- Implement access control lists (ACL) for topics
- Rotate authentication credentials regularly
- Monitor for unauthorized connections

### Reliability
- Deploy redundant Zenoh routers
- Use persistent storage for critical data
- Implement health checking
- Plan for network partitions

### Performance
- Use shared memory for local communication
- Tune batch sizes for your network
- Monitor latency and throughput
- Profile with Zenoh tools

## Resources

- **Official Tutorial**: [Autoware with RMW Zenoh](https://github.com/evshary/autoware_rmw_zenoh)
- **Zenoh Documentation**: [https://zenoh.io/docs/](https://zenoh.io/docs/)
- **ROS 2 RMW Zenoh**: [https://github.com/ros2/rmw_zenoh](https://github.com/ros2/rmw_zenoh)
- **Performance Benchmarks**: [Zenoh Performance Study](https://zenoh.io/blog/2021-07-13-zenoh-performance-async/)

## Next Steps

- Configure platform-specific optimizations for [x86](../x86-based_ECU/index.md) or [ARM](../ARM-based_ECU/index.md)
- Implement fleet management with Zenoh routers
- Set up cloud connectivity for remote monitoring
- Explore edge computing patterns with Zenoh