# Drone Core Demo Scripts

This directory contains demonstration scripts that showcase the drone-core capabilities.

## Available Demos

### 1. Basic Sortie (`run_sortie.py`)

Demonstrates basic autonomous missions without vision detection.

**Features:**
- Perimeter patrol mission
- Area surveillance mission
- Autonomous takeoff, navigation, and landing
- Mission reporting

**Usage:**
```bash
# Run perimeter patrol
python scripts/run_sortie.py --mission perimeter_patrol

# Run area search
python scripts/run_sortie.py --mission area_search

# With GUI
python scripts/run_sortie.py --mission perimeter_patrol --gui
```

**Requirements:**
- pybullet
- numpy
- All core drone-core dependencies

### 2. Vision Demo (`vision_demo.py`)

Enhanced demo with YOLO object detection and threat identification.

**Features:**
- Real-time object detection using YOLO
- Multi-object tracking
- Threat classification (person detection)
- Vision statistics and FPS monitoring
- Annotated video output
- Simulated environment with objects

**Usage:**
```bash
# Run with vision detection (no GUI)
python scripts/vision_demo.py --vision

# Run with GUI and vision
python scripts/vision_demo.py --gui --vision

# Run simulation only (no vision)
python scripts/vision_demo.py --gui
```

**Requirements:**
- All basic requirements
- ultralytics (for YOLO)
- opencv-python
- torch (optional, for GPU acceleration)

**Installation:**
```bash
pip install ultralytics torch torchvision
```

## Demo Comparison

| Feature | run_sortie.py | vision_demo.py |
|---------|---------------|----------------|
| Autonomous flight | ✅ | ✅ |
| Mission planning | ✅ | ❌ |
| Object detection | ❌ | ✅ |
| Threat tracking | ❌ | ✅ |
| Scene with objects | ❌ | ✅ |
| Vision statistics | ❌ | ✅ |
| Annotated video | ❌ | ✅ |

## Testing Workflow

1. **Start with basic tests:**
   ```bash
   # Verify installation
   python -m pytest tests/ -v

   # Test basic simulation
   python scripts/run_sortie.py --mission perimeter_patrol
   ```

2. **Test with GUI:**
   ```bash
   # Visualize the drone flight
   python scripts/run_sortie.py --mission perimeter_patrol --gui
   ```

3. **Test vision system:**
   ```bash
   # Install YOLO first
   pip install ultralytics

   # Run vision demo
   python scripts/vision_demo.py --vision
   ```

4. **Full integration test:**
   ```bash
   # Run with all features
   python scripts/vision_demo.py --gui --vision
   ```

## Expected Output

### run_sortie.py
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
...
----------------------------------------------------------------------

MISSION COMPLETE
======================================================================
Status: COMPLETED
Duration: 125.3 seconds
Distance Traveled: 245.8 meters
Threats Detected: 0
```

### vision_demo.py (with --vision)
```
======================================================================
DRONE CORE - VISION SYSTEM DEMO
======================================================================

[1/6] Initializing simulation...
[2/6] Connecting to drone...
[3/6] Adding objects to scene...
Added 5 objects to scene
[4/6] Initializing vision system...
Vision system ready (YOLO loaded)
[5/6] Arming drone...
[6/6] Starting vision demo...

----------------------------------------------------------------------

Flying to waypoint 1/8: [0, 0, 10]
  Frame 3: 2 detections, 1 threats, FPS: 12.3
  Frame 5: 2 detections, 1 threats, FPS: 13.1
...

----------------------------------------------------------------------

VISION DEMO COMPLETE
======================================================================
Frames Processed: 80
Total Detections: 156
Total Threats: 23
Average FPS: 14.2
```

## Troubleshooting

### PyBullet GUI Issues
```bash
# Linux - allow X11
xhost +local:docker

# Or run headless
python scripts/run_sortie.py --mission perimeter_patrol
# (without --gui)
```

### YOLO Download
On first run, YOLO will auto-download models (~6MB). This may take a minute.

### Performance
- Without GUI: ~30 FPS simulation
- With GUI: ~20 FPS simulation
- With vision: ~10-15 FPS (CPU), ~30+ FPS (GPU)

### Memory Usage
- Basic simulation: ~200MB
- With YOLO: ~500MB (nano model), ~2GB (larger models)

## Next Steps

1. **Create custom scenarios** - Add your own objects and test cases
2. **Optimize detection** - Tune confidence thresholds and model sizes
3. **Hardware integration** - Test with real drone using HardwareBackend
4. **Advanced missions** - Combine vision with autonomous mission planning

## Contributing

To add a new demo:
1. Create script in `scripts/` directory
2. Follow the naming convention: `<feature>_demo.py`
3. Add usage instructions to this README
4. Include example output
5. Update test suite if needed
