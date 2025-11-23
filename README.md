# Drone Core - Autonomous Security Drone System

An autonomous security drone platform with simulation-first development approach. This project provides a complete software stack for autonomous patrol, surveillance, and intrusion detection that can run in simulation or deploy to real hardware.

## 🚁 Overview

**Drone Core** is designed to enable autonomous security operations including:
- **Autonomous Patrol**: Pre-defined or dynamic patrol routes
- **Intrusion Detection**: Computer vision-based threat detection
- **Area Surveillance**: Persistent monitoring of designated zones
- **Autonomous Navigation**: GPS-denied and GPS-enabled navigation
- **Mission Planning**: Define and execute complex sortie missions

## 🏗️ Architecture

### Hardware Abstraction Layer (HAL)
The HAL provides a unified interface that works identically in simulation and on real hardware:
- **Simulation Backend**: Gazebo/PyBullet physics simulation
- **Hardware Backend**: PX4/ArduPilot flight controller interface

### Core Components
1. **Vision System**: OpenCV + YOLO-based computer vision
2. **Navigation System**: Autonomous path planning and obstacle avoidance
3. **Control System**: High-level mission control and low-level flight control
4. **Mission Planner**: Define patrol routes and sortie objectives

### Technology Stack
- **Python 3.10+**: Primary development language
- **ROS2 Humble** (Optional): Message passing and modularity
- **OpenCV**: Computer vision processing
- **PyTorch/ONNX**: ML inference for object detection
- **MAVLink**: Flight controller communication
- **Docker**: Containerized simulation environment

## 🚀 Quick Start - Run a Simulated Sortie

### Prerequisites
- Docker and Docker Compose
- (Optional) NVIDIA GPU with Docker GPU support for ML inference

### Launch Simulation Environment
```bash
# Clone the repository
git clone https://github.com/cbizz2050/drone-core.git
cd drone-core

# Build and launch the simulation
docker-compose up

# In another terminal, launch a test sortie
docker exec -it drone-core-sim python scripts/run_sortie.py --mission perimeter_patrol
```

The drone will autonomously execute a perimeter patrol mission in the simulated environment.

## 📁 Project Structure

```
drone-core/
├── src/drone_core/          # Core drone software
│   ├── hal/                 # Hardware Abstraction Layer
│   ├── vision/              # Computer Vision modules
│   ├── navigation/          # Navigation and path planning
│   ├── control/             # Flight control system
│   └── missions/            # Mission planning and execution
├── simulation/              # Simulation environments
│   ├── worlds/              # Gazebo/simulation world files
│   ├── models/              # Drone and object 3D models
│   └── scenarios/           # Test scenarios and sorties
├── docker/                  # Docker configuration
├── config/                  # Configuration files
├── tests/                   # Unit and integration tests
├── scripts/                 # Utility scripts
└── docs/                    # Documentation
```

## 🎯 Simulation-to-Hardware Workflow

1. **Develop in Simulation**: Write and test all code in the Docker simulation
2. **Validate Performance**: Run test sorties and validate CV/navigation
3. **Deploy to Hardware**: Same code runs on real drone with hardware HAL backend
4. **Field Testing**: Incremental real-world testing with safety protocols

## 🛠️ Development

### Install for Local Development
```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
pip install -e .

# Run tests
pytest tests/
```

### Running Individual Components
```bash
# Test computer vision system
python -m drone_core.vision.detector --source simulation

# Test navigation system
python -m drone_core.navigation.planner --visualize

# Run manual flight test
python scripts/manual_control.py
```

## 📋 Mission Examples

### Perimeter Patrol
```python
from drone_core.missions import PerimeterPatrol

mission = PerimeterPatrol(
    waypoints=[(0,0,10), (100,0,10), (100,100,10), (0,100,10)],
    altitude=10,
    speed=5,
    detect_intruders=True
)
mission.execute()
```

### Area Surveillance
```python
from drone_core.missions import AreaSurveillance

mission = AreaSurveillance(
    zone=[(0,0), (100,0), (100,100), (0,100)],
    altitude=15,
    pattern='grid',
    threat_detection=True
)
mission.execute()
```

## 🔧 Configuration

Configuration files in `config/`:
- `drone_config.yaml`: Drone physical parameters
- `vision_config.yaml`: Computer vision settings
- `mission_config.yaml`: Mission parameters
- `simulation_config.yaml`: Simulation environment settings

## 🧪 Testing

```bash
# Run all tests
pytest

# Run specific test suite
pytest tests/test_vision.py
pytest tests/test_navigation.py

# Run simulation integration tests
pytest tests/integration/test_sortie.py
```

## 📚 Documentation

- [Architecture Overview](docs/architecture.md)
- [Hardware Setup Guide](docs/hardware_setup.md)
- [Simulation Guide](docs/simulation.md)
- [Mission Planning](docs/mission_planning.md)
- [API Reference](docs/api_reference.md)

## 🎓 Hardware Compatibility

Designed to work with common drone hardware:
- **Flight Controllers**: PX4, ArduPilot (Pixhawk family)
- **Companion Computers**: Raspberry Pi 4, Jetson Nano/Xavier
- **Cameras**: USB webcams, CSI cameras, Intel RealSense
- **Platforms**: Quadcopters (450-650mm class recommended)

## 🔒 Security Considerations

This is a security drone platform. Please note:
- Use only in authorized areas
- Comply with local drone regulations (FAA Part 107, etc.)
- Implement proper safety protocols
- Geofencing and fail-safes are critical for real deployments

## 📄 License

MIT License - See [LICENSE](LICENSE) file for details

## 🤝 Contributing

Contributions welcome! Please read [CONTRIBUTING.md](docs/CONTRIBUTING.md) for guidelines.

## 📧 Contact

Project maintained by Cbizz - Security Drone Automation

---

**Status**: 🚧 In Active Development - Simulation environment ready for testing
