# Quick Start Guide

Get up and running with Drone Core in minutes!

## Prerequisites

### For Docker (Recommended)
- Docker and Docker Compose installed
- (Optional) NVIDIA Docker for GPU acceleration

### For Local Development
- Python 3.10 or higher
- pip and virtualenv
- (Optional) CUDA for GPU acceleration

## Installation

### Option 1: Docker (Easiest)

```bash
# Clone the repository
git clone https://github.com/cbizz2050/drone-core.git
cd drone-core

# Build and run simulation
docker-compose up drone-sim
```

That's it! The simulation will start and run a test perimeter patrol mission.

### Option 2: Local Installation

```bash
# Clone the repository
git clone https://github.com/cbizz2050/drone-core.git
cd drone-core

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Install package in development mode
pip install -e .
```

## Running Your First Mission

### Using the Run Script

```bash
# Run perimeter patrol (no GUI)
python scripts/run_sortie.py --mission perimeter_patrol

# Run perimeter patrol with PyBullet GUI
python scripts/run_sortie.py --mission perimeter_patrol --gui

# Run area search mission
python scripts/run_sortie.py --mission area_search --gui
```

### Using Docker

```bash
# Run default mission
docker-compose up drone-sim

# Run with custom mission
docker-compose run drone-sim python scripts/run_sortie.py --mission area_search --gui

# Interactive mode
docker-compose run drone-sim bash
# Then inside container:
python scripts/run_sortie.py --mission perimeter_patrol --gui
```

## Writing Your First Custom Mission

Create a file `my_mission.py`:

```python
import asyncio
from drone_core.hal.simulation import SimulationBackend
from drone_core.vision.system import VisionSystem
from drone_core.navigation.system import NavigationSystem
from drone_core.missions import PerimeterPatrol

async def run_custom_mission():
    # Create simulation drone
    drone = SimulationBackend({
        "use_gui": True,
        "start_position": [0, 0, 0.5]
    })

    # Connect
    await drone.connect()

    # Create systems
    vision = VisionSystem({
        "detector": {"model_type": "yolo"},
        "threat_classes": ["person"]
    })

    navigation = NavigationSystem(drone, {
        "planner": {"default_altitude": 10.0}
    })

    # Define mission
    mission = PerimeterPatrol(
        waypoints=[(0,0), (50,0), (50,50), (0,50)],
        altitude=10.0,
        speed=5.0,
        num_loops=1
    )

    # Execute
    result = await mission.execute(drone, vision, navigation)

    print(f"Mission complete: {result.status}")

    # Cleanup
    await drone.disconnect()

# Run it
asyncio.run(run_custom_mission())
```

Run it:
```bash
python my_mission.py
```

## Using Configuration Files

Create a mission YAML file `my_patrol.yaml`:

```yaml
type: perimeter_patrol

drone:
  connection_string: "udp:127.0.0.1:14550"

vision:
  detector:
    model_type: "yolo"
    confidence_threshold: 0.5
  threat_classes: ["person", "car"]

navigation:
  planner:
    default_altitude: 12.0
    default_speed: 6.0

params:
  waypoints:
    - [0, 0]
    - [80, 0]
    - [80, 80]
    - [0, 80]
  altitude: 12.0
  speed: 6.0
  detect_intruders: true
  num_loops: 3
```

Run it:
```bash
python -m drone_core.cli mission --mission my_patrol.yaml --simulate --gui
```

## Understanding the Output

When you run a mission, you'll see:

```
======================================================================
DRONE CORE - PERIMETER PATROL SORTIE
======================================================================

[1/5] Initializing simulation...
[2/5] Connecting to drone...
[3/5] Initializing vision and navigation systems...
[4/5] Creating mission...
[5/5] Executing sortie...

----------------------------------------------------------------------
[Test Perimeter Patrol] Arming drone...
[Test Perimeter Patrol] Taking off to 10.0m...
[Test Perimeter Patrol] Starting patrol...
[Test Perimeter Patrol] Patrol loop 1/2
[Test Perimeter Patrol] Patrol loop 2/2
[Test Perimeter Patrol] Landing...
----------------------------------------------------------------------

MISSION COMPLETE
======================================================================
Status: COMPLETED
Duration: 124.3 seconds
Distance Traveled: 320.5 meters
Threats Detected: 0
```

## Next Steps

### 1. Explore Example Missions
Check out `config/missions/` for example mission configurations.

### 2. Customize Vision System
Edit `config/vision_config.yaml` to:
- Change detection confidence thresholds
- Modify threat classes
- Tune tracking parameters

### 3. Create Custom Patrol Routes
Define your own waypoints and patrol patterns.

### 4. Add Computer Vision
Enable YOLO detection:
```bash
# Install YOLO
pip install ultralytics

# Models auto-download on first use
# Or manually download: YOLOv8n, YOLOv8s, etc.
```

### 5. Prepare for Hardware
When ready to deploy to real drone:

1. Configure hardware connection in `config/drone_config.yaml`
2. Set up MAVLink connection (USB, UDP, or TCP)
3. Connect camera (USB or CSI)
4. Test connection:
```python
from drone_core.hal.hardware import HardwareBackend

drone = HardwareBackend({
    "connection_string": "/dev/ttyUSB0",
    "baud_rate": 57600
})
await drone.connect()
```

## Troubleshooting

### PyBullet GUI not showing
```bash
# Linux: Allow X11 forwarding
xhost +local:docker

# Or run without GUI
python scripts/run_sortie.py --mission perimeter_patrol
```

### ImportError for pybullet
```bash
pip install pybullet
```

### YOLO model download fails
```bash
# Manually download YOLOv8n
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
```

### Docker permission denied
```bash
sudo usermod -aG docker $USER
# Logout and login again
```

## Getting Help

- **Documentation**: Check `docs/` directory
- **Examples**: See `config/missions/` for mission examples
- **Issues**: Report bugs on GitHub
- **Architecture**: Read `docs/architecture.md`

## What's Next?

Now that you've run your first mission, explore:

- [Architecture Overview](architecture.md) - Understanding the system
- [Mission Planning](mission_planning.md) - Creating complex missions
- [Hardware Setup](hardware_setup.md) - Deploying to real drone
- [API Reference](api_reference.md) - Detailed API documentation

Happy flying! 🚁
