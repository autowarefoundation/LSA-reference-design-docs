# Docker Container Configuration on AGX Orin

This section provides comprehensive documentation for AutoSDV's containerized development environment, specifically optimized for NVIDIA Jetson and AGX Orin platforms. The AutoSDV project implements a modern containerized approach to Autoware development that streamlines both development and deployment workflows.

## Overview

AutoSDV (Autonomous Software Development Vehicle) provides a complete containerized development pipeline that follows the principle of "write once, run anywhere" for Autoware-based autonomous vehicle development. The system is specifically optimized for NVIDIA Jetson hardware platforms and provides a unified development environment.

## Documentation Structure

### 1. [Environment Overview](1-autosdv-environment-overview.md)
**Core Pipeline & Philosophy**

Learn about AutoSDV's fundamental approach to containerized development:
- Containerized compilation pipeline
- Debian packaging system
- Ansible-based provisioning
- Reuse principles for faster deployment

### 2. [Containerized Development Guide](2-autoadv-with-containerization.md)
**Developer Setup & Workflow**

Complete developer guide covering:
- Prerequisites (Docker Engine, NVIDIA Container Toolkit, QEMU)
- Dockerfile analysis with Jetson-optimized base images
- Core development workflow using Makefile
- CUDA support strategies

### 3. [Environment Customization](3-customize-package.md)
**Ansible-Based Configuration**

Advanced customization techniques:
- Ansible provisioning strategies
- Creating custom roles and playbooks
- Modular architecture principles
- Best practices for contributions

### 4. [Debian Package Deployment](4-autoware-deb-package.md)
**Production Deployment**

End-user deployment strategies:
- Local APT repository configuration
- Package build system
- Core deployment scripts
- Distribution strategies for stable releases

## Quick Start

If you're new to AutoSDV's containerized environment:

1. **Start with [Environment Overview](1-autosdv-environment-overview.md)** to understand the overall architecture
2. **Follow the [Containerized Development Guide](2-autoadv-with-containerization.md)** to set up your development environment
3. **Customize your setup** using the [Environment Customization](3-customize-package.md) guide
4. **Deploy to production** using the [Debian Package Deployment](4-autoware-deb-package.md) strategy

## Key Benefits

- **Hardware Optimized**: Specifically designed for NVIDIA Jetson/AGX Orin platforms
- **Unified Environment**: Consistent development experience across different hardware
- **Modular Architecture**: Ansible-based customization for flexible configurations
- **Production Ready**: Complete deployment pipeline from development to production
- **Performance Focused**: Optimized Docker images with CUDA support