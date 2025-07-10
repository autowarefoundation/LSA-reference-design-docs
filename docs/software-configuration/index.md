# Software Configuration

This section provides comprehensive guidance for deploying Autoware on Low Speed Autonomy (LSA) vehicles. The documentation covers everything from initial system setup to platform-specific optimizations.

## Getting Started

This section provides the essential foundation for deploying Autoware on any platform. It covers:

- System requirements (Ubuntu 22.04 on AMD64/ARM64)
- Base system preparation and package installation
- CUDA driver requirements and setup guidance
- Autoware installation methods (source or Debian packages)
- System verification and troubleshooting

Start here: [Getting Started Guide](getting-started/index.md)

## Platform-Specific ECU Deployment

Given the containerized Autoware, this section provides the instructions to deploy the containers on different types of ECUs, including X86-based and ARM-based ECUs.

### Instructions to deploy Autoware on x86-based ECUs

X86-based ECUs are well-received for research, development, and PoC. Several tested X86-based ECUs are listed on [this page](../hardware-configuration/ECUs/x86ECUs/index.md)

The instructions to deploy containerized Autoware include the following topics: 

- x86 hardware requirements and compatibility
- CUDA setup for discrete GPUs (NVIDIA RTX/Tesla)
- Platform-specific optimizations
- Custom configurations for x86 architectures

Read the instructions to deploy containerized Autotware to [x86-based ECUs](x86-based_ECU/index.md)

### Instructions to deploy Autoware on ARM-based ECUs

ARM-based ECUs are well received for low-energy consumption vehicles and several of them are tested and listed on this [page](../hardware-configuration/ECUs/armECUs/index.md).

The instructions to deploy containerized Autoware include the following topics: 

- ARM platform specifications (AGX Orin, Xavier, etc.)
- Integrated GPU optimization
- [Containerized development workflow](ARM-based_ECU/containerized-development.md)
- [Platform-specific customizations](ARM-based_ECU/customization.md)

Read the instructions to deploy containerized Autotware to [ARM-based ECUs](ARM-based_ECU/index.md)

## Sensor Configuration

Before deploying Autoware on any ECU, sensors must be properly configured. This section covers:

- Network configuration for Ethernet-based sensors
- LiDAR setup (Velodyne, Ouster, Hesai)
- Camera configuration (USB, GMSL, GigE Vision)
- CAN bus interface for vehicle communication
- GNSS/IMU integration
- Time synchronization and calibration

Read the comprehensive [Sensor Configuration Guide](sensor-configuration/index.md)
## Middleware Configuration

ROS 2 supports multiple middleware implementations beyond the default DDS. This section covers:

- Understanding different RMW (ROS Middleware) options
- Why Zenoh is recommended for LSA applications
- Performance benefits for wireless and cellular networks
- Configuration for both single-vehicle and fleet deployments
- Easy switching between middleware implementations

Learn more: [Middleware Configuration](middleware-configuration/index.md)

# Deployment Workflow

Follow this recommended workflow for deploying Autoware on your LSA vehicle:

1. **System Setup**: Start with [Getting Started](getting-started/index.md) to prepare your base system
2. **Platform Configuration**: Choose your platform-specific guide:
   - [x86 ECUs](x86-based_ECU/index.md) for Intel/AMD systems
   - [ARM ECUs](ARM-based_ECU/index.md) for NVIDIA Jetson platforms
3. **Sensor Integration**: Configure your sensors using the [Sensor Configuration Guide](sensor-configuration/index.md)
4. **Middleware Optimization**: Consider [alternative middleware](middleware-configuration/index.md) for better performance
5. **Fine-tuning**: Apply platform-specific customizations for your use case

# Support and Resources

Below are the support and resources for the software configuration guideline.

- **Autoware Documentation**: [https://autoware.org/](https://autoware.org/)
- **ROS 2 Documentation**: [https://docs.ros.org/](https://docs.ros.org/)
- **NVIDIA Jetson Resources**: [https://developer.nvidia.com/embedded-computing](https://developer.nvidia.com/embedded-computing)
