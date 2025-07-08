# Software Configuration

This section provides comprehensive guidance for deploying Autoware on Low Speed Autonomy (LSA) vehicles. The documentation covers everything from initial system setup to platform-specific optimizations.

# Struture of the Guidelines

The guidelines consist of several parts. You may go directly to [Getting Started](#getting-started) if you want to skip the structure of the guidelines.

## Preparation for Deployment

This guideline uses a containerized Autoware for deployment, which is hardeware independent. Hence, this guideline can be used for both X86 and ARM-based ECUs. The guideline inlcudes the following topics: 

- Target environment requirements (Ubuntu 22.04 on AMD64/ARM64)
- CUDA driver installation and verification
- Ansible-based provisioning setup
- Autoware installation via Debian packages
- Frequent troubleshooting and verification

Read [Preparation for Deployment](deployment-setup/index.md)

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

Read the instructions to deploy containerized Autotware to [x86-based ECUs](x86-based_ECU/index.md)

## RMW Zenoh as the middleware for ROS 2

The middleware in ROS 2 have several options. DDS is the default configuration and rmw_zenoh has been approved by ROS 2 to be compatiable as the middleware of ROS 2. The instructions show how to use rmw_zenoh as the middleware of ROS 2. rmw_zenoh has the following features:

- Alternative to DDS with improved performance
- Simplified configuration for LSA applications
- Enhanced support for cellular and cloud connectivity
- Optimized for resource-constrained environments

Read the instructions to deploy [rmw_zenoh](rmw_zenoh/index.md)

# Getting Started

1. **Review Requirements**: Start with [Deployment Setup](deployment-setup/index.md) to understand system requirements
2. **Choose Your Platform**: Select either x86 or ARM ECUs based on your design requirements. 
3. **Install Autoware**: Follow the platform-specific installation guide: [x86](x86-based_ECU/index.md) or [ARM](ARM-based_ECU/index.md).
4. **Configure Middleware**: Optionally switch to [rmw_zenoh](rmw_zenoh/index.md) to improve the performance of ROS messages.
5. **Customize the system**: Apply platform-specific optimizations for your use case

# Support and Resources

Below are the support and resources for the software configuration guideline.

- **Autoware Documentation**: [https://autoware.org/](https://autoware.org/)
- **ROS 2 Documentation**: [https://docs.ros.org/](https://docs.ros.org/)
- **NVIDIA Jetson Resources**: [https://developer.nvidia.com/embedded-computing](https://developer.nvidia.com/embedded-computing)

# Contributing
When contributing to this documentation:

1. Follow the existing structure and formatting
2. Test all commands and procedures on target hardware
3. Include troubleshooting sections for common issues
4. Keep content up-to-date with latest Autoware releases
