# AutoSDV Containerized Development Environment

This guide walks developers through AutoSDV's containerized development environment, specifically optimized for NVIDIA Jetson (ARM64) platforms.

## Development Philosophy

AutoSDV's containerized approach eliminates the complexity of environment setup by treating infrastructure as code. The entire development environment is defined in a `Dockerfile`, ensuring every developer works with an identical, reproducible setup regardless of their host system.

## Prerequisites

Before getting started, ensure you have the following installed:

- **Docker Engine**: Container runtime environment
- **NVIDIA Container Toolkit**: GPU acceleration support for containers
- **QEMU**: Cross-architecture build support (required for x86_64 hosts)
- **Git**: Version control for repository management

## Quick Start Workflow

The development workflow is streamlined through a `Makefile` located in the `docker/` directory:

### 1. Repository Setup
```bash
git clone -b 2025.02 --recursive https://github.com/NEWSLabNTU/AutoSDV.git
cd AutoSDV/docker
```

### 2. Bootstrap Environment (First Time Only)
Configures QEMU for cross-architecture development:
```bash
make bootstrap
```

### 3. Build Development Image
Creates the ARM64 Docker image with commit-based tagging for traceability:
```bash
make build
```

### 4. Launch Development Container
Starts an interactive container with live code synchronization:
```bash
make run
```

**Note**: The `make run` command automatically mounts your local `./src` directory to `/home/developer/AutoSDV/src` in the container, enabling real-time code editing from your host machine.

## Technical Implementation

### Container Architecture

The AutoSDV container is built on several key design principles:

#### Base Image Strategy
- **Foundation**: `nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel`
- **Benefits**: Jetson-optimized environment with pre-installed CUDA and TensorRT stack
- **Target**: Specifically designed for NVIDIA Jetson hardware acceleration

#### Build Reproducibility
- **Commit Tagging**: `COMMIT_HASH` build argument creates precise code snapshots
- **Version Control**: Every image is traceable to a specific repository state
- **Consistency**: Guarantees identical environments across all development machines

#### Enhanced Package Access
- **APT Enhancement**: Custom `nvidia-l4t-apt-source.list` provides access to additional NVIDIA packages
- **Extended Capabilities**: Includes specialized tools like `nvidia-l4t-dla-compiler`
- **Optimized Stack**: Full GPU acceleration toolkit for Jetson platforms

#### Automated Provisioning
- **Entry Point**: `setup-dev-env.sh` script orchestrates environment setup
- **Ansible Integration**: Automated provisioning ensures consistent configuration
- **Zero-Configuration**: Complete development environment ready immediately

### CUDA Support Comparison

| Approach | AutoSDV | OpenADKit |
|----------|---------|----------|
| **Strategy** | Implicit Integration | Explicit Configuration |
| **Base Image** | `l4t-tensorrt` (Jetson-native) | Multiple tagged variants |
| **CUDA Support** | Built-in, optimized | Optional, flexible |
| **Target Use Case** | NVIDIA Jetson platforms | Multi-platform support |
| **Setup Complexity** | Zero-configuration | Manual selection required |

**AutoSDV Advantage**: By choosing a Jetson-native foundation, AutoSDV delivers an optimized, ready-to-use experience specifically tailored for NVIDIA embedded platforms, eliminating configuration overhead while maximizing performance.