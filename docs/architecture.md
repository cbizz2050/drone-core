# Drone Core - Architecture Overview

## System Architecture

Drone Core follows a layered architecture with clear separation between simulation and hardware interfaces.

```
┌─────────────────────────────────────────────────────────┐
│                    Mission Layer                        │
│  (PerimeterPatrol, AreaSurveillance, Custom Missions)  │
└─────────────────────────────────────────────────────────┘
                           │
                           ↓
┌─────────────────────────────────────────────────────────┐
│                  High-Level Systems                     │
├──────────────────────┬──────────────────────────────────┤
│   Vision System      │    Navigation System             │
│  - Object Detection  │   - Path Planning                │
│  - Object Tracking   │   - Waypoint Navigation          │
│  - Threat Detection  │   - Flight Control               │
└──────────────────────┴──────────────────────────────────┘
                           │
                           ↓
┌─────────────────────────────────────────────────────────┐
│          Hardware Abstraction Layer (HAL)               │
│              DroneInterface (Abstract)                  │
└─────────────────────────────────────────────────────────┘
                           │
                ┌──────────┴──────────┐
                ↓                     ↓
┌───────────────────────┐  ┌─────────────────────┐
│  SimulationBackend    │  │  HardwareBackend    │
│   - PyBullet Physics  │  │  - MAVLink Protocol │
│   - Virtual Camera    │  │  - Real Camera      │
│   - Virtual Sensors   │  │  - Real Sensors     │
└───────────────────────┘  └─────────────────────┘
```

## Core Components

### 1. Hardware Abstraction Layer (HAL)

**Purpose**: Provide unified interface for both simulation and real hardware.

**Key Classes**:
- `DroneInterface`: Abstract base class defining drone control API
- `SimulationBackend`: PyBullet-based simulation implementation
- `HardwareBackend`: MAVLink-based real hardware implementation

**Benefits**:
- Same code runs in simulation and on hardware
- Easy testing and development
- Swap backends without changing mission code

### 2. Vision System

**Purpose**: Computer vision for object detection and threat identification.

**Key Classes**:
- `ObjectDetector`: ML-based object detection (YOLO, ONNX, OpenCV)
- `ObjectTracker`: Multi-object tracking across frames
- `VisionSystem`: Integrated vision pipeline with threat assessment

**Capabilities**:
- Real-time object detection
- Multi-object tracking
- Threat classification
- Visual alerts and logging

### 3. Navigation System

**Purpose**: Autonomous path planning and waypoint navigation.

**Key Classes**:
- `PathPlanner`: Generate paths for various patterns (grid, perimeter, spiral, orbit)
- `NavigationController`: Execute waypoint navigation
- `NavigationSystem`: High-level navigation commands

**Features**:
- Multiple search patterns
- Waypoint optimization
- Path interpolation
- Flight time estimation

### 4. Mission System

**Purpose**: High-level mission definition and execution.

**Key Classes**:
- `Mission`: Abstract mission base class
- `PerimeterPatrol`: Patrol perimeter boundaries
- `AreaSurveillance`: Systematic area search

**Features**:
- Mission state management
- Pause/resume/abort control
- Result tracking and reporting
- Vision integration

## Data Flow

### Typical Mission Execution Flow

```
1. Mission Initialization
   └─> Load configuration
   └─> Create drone interface (Simulation or Hardware)
   └─> Initialize vision and navigation systems

2. Mission Preparation
   └─> Connect to drone
   └─> Validate systems
   └─> Set home position

3. Mission Execution
   └─> Arm drone
   └─> Takeoff
   └─> Execute waypoint navigation
       ├─> Vision processing (async)
       │   └─> Object detection
       │   └─> Object tracking
       │   └─> Threat identification
       └─> Navigation control
           └─> Send position commands
           └─> Monitor progress

4. Mission Completion
   └─> Return to home (optional)
   └─> Land
   └─> Disarm
   └─> Generate mission report

5. Cleanup
   └─> Disconnect systems
   └─> Save logs/data
```

## Configuration System

Configuration is hierarchical and modular:

```
config/
├── drone_config.yaml       # Drone physical parameters
├── vision_config.yaml      # Vision system settings
└── missions/
    ├── example_patrol.yaml      # Perimeter patrol config
    └── example_surveillance.yaml # Area search config
```

Each system can be configured independently, allowing for:
- Easy tuning and optimization
- Mission templates
- Hardware-specific configurations
- Environment-specific settings

## Extensibility

### Adding New Mission Types

1. Subclass `Mission` base class
2. Implement `execute()` method
3. Define mission-specific parameters
4. Use existing vision/navigation systems

Example:
```python
class CustomMission(Mission):
    async def execute(self, drone, vision, nav):
        # Your mission logic here
        pass
```

### Adding New Detection Models

1. Implement detector in `ObjectDetector` class
2. Add new `model_type` option
3. Configure in `vision_config.yaml`

### Adding New Navigation Patterns

1. Add method to `PathPlanner` class
2. Return list of `Waypoint` objects
3. Use in missions via `NavigationSystem`

## Simulation to Hardware Workflow

```
┌──────────────┐
│ Development  │  Develop and test in simulation
│ (Simulation) │  - Fast iteration
└──────┬───────┘  - No hardware risk
       │
       ↓
┌──────────────┐
│ Validation   │  Validate algorithms and logic
│ (Simulation) │  - Complete mission runs
└──────┬───────┘  - Performance testing
       │
       ↓
┌──────────────┐
│ Hardware     │  Deploy exact same code to hardware
│  Integration │  - Switch backend only
└──────┬───────┘  - Verify hardware interfaces
       │
       ↓
┌──────────────┐
│ Field        │  Real-world testing
│  Testing     │  - Safety protocols
└──────────────┘  - Incremental validation
```

## Safety Features

1. **Geofencing**: Configurable boundaries
2. **Battery Monitoring**: Auto-RTL on low battery
3. **Emergency Stop**: Immediate motor cutoff
4. **Fail-safes**: RTL on communication loss
5. **Altitude Limits**: Min/max altitude enforcement

## Performance Considerations

### Simulation Performance
- Adjust timestep for accuracy vs. speed
- Use headless mode for faster execution
- GPU acceleration for ML inference

### Hardware Performance
- Consider onboard compute limitations
- Optimize ML models (ONNX, quantization)
- Balance vision FPS with flight control rate

## Future Enhancements

- [ ] ROS2 integration for modularity
- [ ] Multi-drone coordination
- [ ] Advanced obstacle avoidance
- [ ] SLAM for GPS-denied navigation
- [ ] Web-based mission control interface
- [ ] Machine learning for threat prediction
- [ ] Automated mission planning from objectives
