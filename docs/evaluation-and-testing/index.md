# Evaluation and Testing

(To be completed)

## Expected Performance Metrics for ARM-based ECUs

| Component | AGX Orin | Xavier | Orin Nano |
|-----------|----------|---------|-----------|
| LiDAR Detection | 10 Hz | 8 Hz | 5 Hz |
| Camera Detection | 20 Hz | 15 Hz | 10 Hz |
| Planning | 10 Hz | 10 Hz | 8 Hz |
| Control | 50 Hz | 50 Hz | 50 Hz |
| Power Usage | 40W | 25W | 12W |

## Expected Performance Metrics for X86-based ECUs

| Component | Target FPS | CPU Usage | GPU Usage | Latency |
|-----------|------------|-----------|-----------|---------|
| LiDAR Detection | 10 Hz | 20-30% | 40-60% | <100ms |
| Camera Detection | 15 Hz | 15-25% | 50-70% | <80ms |
| Planning | 10 Hz | 30-40% | N/A | <50ms |
| Control | 50 Hz | 10-15% | N/A | <20ms |

### AdvanTech X86-based ECU

| Component | Target FPS | CPU Usage | GPU Usage | Latency |
|-----------|------------|-----------|-----------|---------|
| LiDAR Detection | 10 Hz | 15-25% | N/A | <150ms |
| Camera Detection | 10 Hz | 5-10% | N/A | <80ms |
| Planning | 10 Hz | 30-45% | N/A | <75ms |
| Control | 20 Hz | 10-25% | N/A | <40ms |

*LiDAR:Use 3D LiDAR Sensor.
*Camera:Use USB camera, resolution is 1280x720.
### vecow X86-based ECU

