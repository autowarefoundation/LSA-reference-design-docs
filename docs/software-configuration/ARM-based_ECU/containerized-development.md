# Container Development

This comprehensive guide covers the containerized development environment for ARM-based ECUs, particularly focused on NVIDIA AGX Orin platforms. The content is adapted from the AutoSDV project, which provides a sophisticated containerized approach to Autoware development.

## Overview

The containerized development approach provides a complete, reproducible environment for developing and deploying Autoware on ARM-based ECUs. This methodology ensures consistency across development teams while optimizing for the specific capabilities of NVIDIA Jetson and AGX Orin hardware.

## Build Philosophy: Compile Once, Deploy Everywhere

The containerized environment follows a four-stage pipeline that maximizes efficiency and reproducibility:

### 1. Containerized Compilation
- **Process**: Complete Autoware build (`colcon build`) within isolated Docker containers
- **Benefits**: 
  - Eliminates host system dependency conflicts
  - Ensures reproducible builds across different development machines
  - Provides consistent compilation environment

### 2. Debian Package Creation
- **Process**: Automated conversion of ROS 2 packages to `.deb` files via `package-deb.sh`
- **Benefits**:
  - Captures compiled artifacts in distributable format
  - Enables version control and release management
  - Supports standard Linux package management

### 3. Ansible-Based Provisioning
- **Process**: Environment setup using pre-built packages instead of source compilation
- **Benefits**:
  - Dramatically faster than full `colcon build`
  - Consistent configuration across deployments
  - Infrastructure-as-code approach

### 4. Unified Docker Environment
- **Process**: Complete pipeline orchestration through Docker
- **Benefits**:
  - Platform-independent development
  - Simplified developer onboarding
  - Consistent toolchain across teams

## Development Environment Setup

### Prerequisites

Before starting, ensure your development machine has:

- **Docker Engine**: Latest version with buildx support
- **NVIDIA Container Toolkit**: For GPU acceleration in containers
- **QEMU**: Cross-architecture build support (for x86_64 development hosts)
- **Git**: For repository management
- **Sufficient Storage**: At least 50GB free space for images and builds

### Quick Start Guide

#### 1. Repository Setup
```bash
# Clone the repository with submodules
git clone --recursive <your-autoware-repository>
cd <repository>/docker
```

#### 2. Bootstrap Environment (First Time Only)
```bash
# Configure QEMU for cross-architecture builds
make bootstrap
```

#### 3. Build Development Image
```bash
# Build ARM64 Docker image with commit tagging
make build
```

#### 4. Launch Development Container
```bash
# Start interactive development session
make run
```

**Note**: The container automatically mounts your local source directory, enabling real-time code synchronization between host and container.

## Technical Architecture

### Container Design Principles

#### Base Image Strategy
The containerized environment uses NVIDIA's official L4T (Linux for Tegra) images as the foundation:

- **Base Image**: `nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel`
- **Includes**: CUDA, cuDNN, TensorRT pre-installed and optimized
- **Target Hardware**: NVIDIA Jetson AGX Orin, Xavier, and compatible platforms

#### Build Reproducibility Features
- **Commit-based tagging**: Each image tagged with Git commit hash
- **Deterministic builds**: Locked dependency versions
- **Build cache optimization**: Layered approach for faster rebuilds
- **Multi-stage builds**: Separate build and runtime environments

#### NVIDIA-Specific Optimizations
- **Custom APT sources**: Access to NVIDIA L4T packages
- **DLA compiler**: Deep Learning Accelerator support
- **Jetson-specific libraries**: Hardware-optimized components
- **Power mode configurations**: Development vs. production profiles

### Container vs. Native Development

| Aspect                    | Containerized            | Native        |
|---------------------------|--------------------------|---------------|
| **Setup Time**            | < 30 minutes             | Several hours |
| **Reproducibility**       | Guaranteed               | Variable      |
| **Dependency Management** | Automated                | Manual        |
| **Hardware Access**       | Full (with proper flags) | Direct        |
| **Performance**           | ~98% of native           | 100%          |
| **Team Scalability**      | Excellent                | Challenging   |

## Environment Customization

### Using Ansible for Configuration Management

The containerized environment uses Ansible for flexible, maintainable configuration:

#### Creating Custom Components

1. **Define Component Role**
```bash
# Create role structure
mkdir -p ansible/roles/my_component/{tasks,defaults,meta}
```

2. **Implement Installation Logic**
```yaml
# ansible/roles/my_component/tasks/main.yml
---
- name: Install component dependencies
  apt:
    name: "{{ component_dependencies }}"
    state: present

- name: Download and install component
  get_url:
    url: "{{ component_url }}"
    dest: "/tmp/{{ component_filename }}"
  
- name: Install component package
  apt:
    deb: "/tmp/{{ component_filename }}"
  when: component_type == "deb"
```

3. **Configure Variables**
```yaml
# ansible/roles/my_component/defaults/main.yml
---
component_name: "my_custom_component"
component_version: "1.0.0"
component_dependencies:
  - libboost-all-dev
  - python3-pip
```

### Hardware-Specific Configurations

#### AGX Orin Optimizations
```yaml
# ansible/roles/agx_orin_config/tasks/main.yml
---
- name: Configure Jetson power mode
  command: nvpmodel -m 0  # MAXN mode for development

- name: Set GPU/DLA clock frequencies
  command: jetson_clocks --fan

- name: Configure memory growth for TensorRT
  lineinfile:
    path: /etc/environment
    line: 'TF_FORCE_GPU_ALLOW_GROWTH=true'
```

## Package Management

### Building Debian Packages

The containerized environment includes sophisticated tooling for creating Debian packages:

#### Individual Package Building
```bash
# Convert single ROS 2 package to .deb
./scripts/make-deb.sh <package_name>
```

#### Batch Package Creation
```bash
# Build all packages in workspace
./scripts/package-deb.sh --workspace /path/to/ws
```

### Repository Management

#### Local APT Repository Structure
```
local-apt-repo/
├── dists/
│   └── focal/
│       └── main/
│           └── binary-arm64/
└── pool/
    └── main/
        └── a/autoware-*/
```

#### Deployment Workflow
1. Build packages in container
2. Export to local repository
3. Configure target ECUs to use repository
4. Deploy via standard `apt` commands

## Development Workflows

### Typical Development Cycle

1. **Code Development**
   - Edit code on host machine
   - Changes reflected instantly in container
   - Use IDE of choice on host

2. **Build and Test**
   ```bash
   # Inside container
   colcon build --packages-select <your_package>
   colcon test --packages-select <your_package>
   ```

3. **Package Creation**
   ```bash
   # Create deployable package
   make package PKG=<your_package>
   ```

4. **Deployment Testing**
   ```bash
   # Test on target hardware
   ansible-playbook deploy.yml -l target_ecu
   ```

### CI/CD Integration

The containerized approach seamlessly integrates with CI/CD pipelines:

```yaml
# Example GitLab CI configuration
build-arm64:
  stage: build
  script:
    - docker buildx build --platform linux/arm64 -t ${IMAGE_TAG} .
    - docker run ${IMAGE_TAG} make test
    - docker run ${IMAGE_TAG} make package
  artifacts:
    paths:
      - packages/*.deb
```

## Performance Optimization

### Container Runtime Optimizations

1. **GPU Access**
   ```bash
   docker run --gpus all --runtime nvidia ...
   ```

2. **Memory Management**
   ```bash
   # Limit container memory to prevent OOM
   docker run -m 16g --memory-swap 16g ...
   ```

3. **CPU Affinity**
   ```bash
   # Pin to specific cores
   docker run --cpuset-cpus="0-3" ...
   ```

### Build Performance

- **Parallel Compilation**: `colcon build --parallel-workers $(nproc)`
- **ccache Integration**: Persistent cache volumes
- **Incremental Builds**: Proper workspace overlays

## Troubleshooting

### Common Issues and Solutions

1. **QEMU Errors on x86_64 Hosts**
   ```bash
   # Re-run bootstrap
   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
   ```

2. **GPU Not Accessible**
   ```bash
   # Verify NVIDIA runtime
   docker run --rm --gpus all nvidia/cuda:11.4.3-base-ubuntu20.04 nvidia-smi
   ```

3. **Build Cache Issues**
   ```bash
   # Clean and rebuild
   docker builder prune
   docker build --no-cache -t ${IMAGE_TAG} .
   ```

## Best Practices

### Development Guidelines

1. **Layer Optimization**
   - Group related operations
   - Minimize layer size
   - Use multi-stage builds

2. **Security Considerations**
   - Don't store credentials in images
   - Use secrets management
   - Regular base image updates

3. **Resource Management**
   - Set appropriate resource limits
   - Monitor container metrics
   - Clean up unused images regularly

### Team Collaboration

1. **Image Versioning**
   - Semantic versioning for releases
   - Commit-based tags for development
   - Maintain changelog

2. **Documentation**
   - Document custom configurations
   - Maintain runbooks
   - Update README files

## Summary

The containerized development approach for ARM-based ECUs provides a robust, scalable solution for Autoware development on NVIDIA Jetson platforms. By combining Docker containerization, Ansible automation, and Debian packaging, teams can achieve:

- **Rapid Development**: From code to deployment in minutes
- **Consistency**: Identical environments across all developers
- **Hardware Optimization**: Full utilization of NVIDIA accelerators
- **Production Ready**: Seamless transition from development to deployment

This methodology has been proven in production environments and continues to evolve with the Autoware ecosystem.
