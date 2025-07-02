# AutoSDV Environment Build Overview

AutoSDV's build system is designed around a core philosophy: **compile once, deploy everywhere**. This approach significantly reduces development time while ensuring consistency across all environments.

## Build Pipeline Architecture

The AutoSDV environment follows a four-stage pipeline that transforms source code into ready-to-use development environments:

### 1. Containerized Compilation

**Process**: Autoware source code undergoes a complete build (`colcon build`) within an isolated Docker container.

**Benefits**: 
- Eliminates host system dependency issues
- Provides a reproducible build environment
- Ensures consistent compilation results across different machines

### 2. Debian Package Creation

**Process**: After successful compilation, the `package-deb.sh` script automatically converts each ROS 2 package into individual `.deb` files using the `make-deb.sh` utility.

**Benefits**:
- Captures build artifacts (binaries, libraries, launch files) in a distributable format
- Creates versionable packages for release management  
- Enables standard Linux package management workflows

### 3. Ansible-Based Environment Provisioning

**Process**: Development environments are provisioned using Ansible playbooks that install the pre-built `.deb` packages instead of compiling from source.

**Benefits**:
- Dramatically faster environment setup compared to full `colcon build`
- Consistent environment configuration across all deployments
- Modular, maintainable infrastructure-as-code approach

### 4. Unified Docker Environment

**Process**: The entire pipeline is orchestrated within Docker containers, accessible through simple `make` commands.

**Benefits**:
- Platform-independent development experience
- Quick environment establishment on any host machine
- Simplified onboarding for new developers

## Summary

AutoSDV leverages three key technologies in sequence:
- **Docker containers** to build binaries in isolation
- **Debian packaging** to create distributable artifacts  
- **Ansible automation** to deploy consistent environments

This pipeline delivers a highly efficient, reproducible workflow that scales from individual development to community-wide deployment.
