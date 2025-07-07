# Software Configuration

This section provides comprehensive guidance for deploying Autoware on Low Speed Autonomy (LSA) vehicles. The documentation covers everything from initial system setup to platform-specific optimizations.

## Documentation Structure

### [Deployment Setup](deployment-setup/index.md)
**Foundation for All Deployments**

Start here for:
- Target environment requirements (Ubuntu 22.04 on AMD64/ARM64)
- CUDA driver installation and verification
- Ansible-based provisioning setup
- Autoware installation via Debian packages
- Common troubleshooting and verification

### Platform-Specific ECU Deployment

#### [x86-based ECU](x86-based_ECU/index.md)
**Intel/AMD Platform Configuration**

Covers:
- x86 hardware requirements and compatibility
- CUDA setup for discrete GPUs (NVIDIA RTX/Tesla)
- Platform-specific optimizations
- Custom configurations for x86 architectures

#### [ARM-based ECU](ARM-based_ECU/index.md)
**NVIDIA Jetson/AGX Orin Platform Configuration**

Includes:
- ARM platform specifications (AGX Orin, Xavier, etc.)
- Integrated GPU optimization
- [Containerized development workflow](ARM-based_ECU/containerized-development.md)
- [Platform-specific customizations](ARM-based_ECU/customization.md)

### [RMW Zenoh Configuration](rmw_zenoh/index.md)
**Next-Generation Middleware for ROS 2**

Features:
- Alternative to DDS with improved performance
- Simplified configuration for LSA applications
- Enhanced support for cellular and cloud connectivity
- Optimized for resource-constrained environments

## Getting Started

1. **Review Requirements**: Start with [Deployment Setup](deployment-setup/index.md) to understand system requirements
2. **Choose Your Platform**: Select either [x86](x86-based_ECU/index.md) or [ARM](ARM-based_ECU/index.md) based on your ECU hardware
3. **Install Autoware**: Follow the platform-specific installation guide
4. **Configure Middleware**: Optionally switch to [rmw_zenoh](rmw_zenoh/index.md) for improved performance
5. **Customize**: Apply platform-specific optimizations for your use case

## Key Considerations

### Hardware Selection
- **x86 ECUs**: Better for high-compute perception tasks, easier software compatibility
- **ARM ECUs**: Superior power efficiency, integrated GPU/DLA acceleration, compact form factor

### Development Approach
- **Native Installation**: Direct installation on hardware for maximum performance
- **Containerized Development**: Reproducible environments with easier team collaboration (recommended for ARM)

### Middleware Choice
- **Default DDS**: Mature, widely supported, industry standard
- **rmw_zenoh**: Modern alternative with better performance for LSA applications

## Support and Resources

- **Autoware Documentation**: [https://autoware.org/](https://autoware.org/)
- **ROS 2 Documentation**: [https://docs.ros.org/](https://docs.ros.org/)
- **NVIDIA Jetson Resources**: [https://developer.nvidia.com/embedded-computing](https://developer.nvidia.com/embedded-computing)

## Contributing

When contributing to this documentation:
1. Follow the existing structure and formatting
2. Test all commands and procedures on target hardware
3. Include troubleshooting sections for common issues
4. Keep content up-to-date with latest Autoware releases