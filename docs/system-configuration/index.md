# System Configuration

(To be completed)

The section describes the system configuration of the LSA vehicles. It shows the options to design your own LSA vehicles. The configuration consists of ECU selection, development approach, middleware candidates, etc. 

# Key Considerations

## Hardware Selection
- **ARM ECUs**: Superior power efficiency, integrated GPU/DLA acceleration, compact form factor

- **x86 ECUs**: Better for high-compute perception tasks, easier software compatibility
### Recommended Hardware Configurations

For X86-based ECUs, there are configurations for different use scearios. 

#### High-Performance Configuration
- **CPU**: Intel Core i9-13900K or AMD Ryzen 9 7950X (16+ cores)
- **RAM**: 64 GB DDR5
- **GPU**: NVIDIA RTX 4090 or RTX A6000
- **Storage**: 1TB NVMe SSD (PCIe 4.0)
- **Network**: Dual 10GbE interfaces
- **Power**: 850W+ PSU with redundancy

#### Standard Configuration
- **CPU**: Intel Core i7-12700K or AMD Ryzen 7 7700X (8+ cores)
- **RAM**: 32 GB DDR4/DDR5
- **GPU**: NVIDIA RTX 3080 or RTX A4000
- **Storage**: 512GB NVMe SSD
- **Network**: Gigabit Ethernet
- **Power**: 650W PSU

#### Minimum Configuration
- **CPU**: Intel Core i5-12600K or AMD Ryzen 5 5600X (6+ cores)
- **RAM**: 16 GB DDR4
- **GPU**: NVIDIA RTX 3060 or T1000
- **Storage**: 256GB SSD
- **Network**: Gigabit Ethernet
- **Power**: 550W PSU

### Industrial/Vehicle-Grade Options
- **Neousys Nuvo-8108GC**: Rugged x86 platform with RTX GPU support
- **Advantech MIC-770**: Compact industrial PC with GPU expansion
- **Crystal Group RS363S**: MIL-SPEC certified with NVIDIA GPU

## Development Approach
- **Native Installation**: Direct installation on hardware for maximum performance
- **Containerized Development**: Reproducible environments with easier team collaboration (recommended for ARM)

## Middleware Choice
- **Default DDS**: Mature, widely supported, industry standard
- **rmw_zenoh**: Modern alternative with better performance for LSA applications


