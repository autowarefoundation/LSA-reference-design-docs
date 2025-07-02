# Software Configuration

This section covers the software configuration aspects for Low Speed Autonomy (LSA) vehicles based on Autoware. The documentation is organized into two main areas:

## rmw_zenoh Middleware

The rmw_zenoh middleware provides a modern alternative to traditional DDS implementations in ROS 2, offering improved performance and simplified configuration for LSA applications.

- **[rmw_zenoh Configuration](rmw_zenoh/index.md)**: Learn about implementing rmw_zenoh as your ROS 2 middleware, including its advantages for low-speed autonomy applications and cellular network support.

## Docker Container Configuration on AGX Orin

AutoSDV provides a comprehensive containerized development environment specifically optimized for NVIDIA Jetson and AGX Orin platforms, enabling efficient development and deployment of Autoware-based systems.

### Development Workflow Documentation

1. **[AutoSDV Environment Overview](docker-container/1-autosdv-environment-overview.md)**: Understanding the core philosophy and pipeline of AutoSDV's containerized build system.

2. **[Containerized Development Guide](docker-container/2-autoadv-with-containerization.md)**: Developer-focused guide covering prerequisites, Docker setup, and NVIDIA Container Toolkit configuration for Jetson platforms.

3. **[Environment Customization](docker-container/3-customize-package.md)**: Learn how to customize your development environment using Ansible for modular, reusable configurations.

4. **[Debian Package Deployment](docker-container/4-autoware-deb-package.md)**: End-user deployment strategy using Debian packages and local APT repositories for production systems.

## Key Features

- **Unified Development Environment**: Docker-based development workflow optimized for NVIDIA Jetson hardware
- **Performance Optimized**: rmw_zenoh middleware for improved ROS 2 communication performance
- **Modular Architecture**: Ansible-based customization for flexible system configuration
- **Production Ready**: Debian packaging system for reliable deployment to end-user systems
- **Hardware Optimized**: Specific optimizations for AGX Orin and Jetson platforms

## Getting Started

For new developers, we recommend starting with the [AutoSDV Environment Overview](docker-container/1-autosdv-environment-overview.md) to understand the overall architecture, then proceeding through the containerized development workflow before exploring middleware configuration options.