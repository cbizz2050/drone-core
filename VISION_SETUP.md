# Vision System Setup Guide

The drone-core project includes an advanced computer vision system for object detection and threat tracking. This guide explains how to set it up.

## Quick Start (Without Vision)

The basic simulation works without vision capabilities:

```bash
# Install core dependencies
pip install -r requirements.txt
pip install -e .

# Run basic simulation
python scripts/run_sortie.py --mission perimeter_patrol
```

## Installing Vision Capabilities

To enable YOLO object detection, install ultralytics:

```bash
# Activate your virtual environment
source venv/bin/activate

# Install ultralytics (includes PyTorch, ~2GB download)
pip install ultralytics

# Optional: Install with GPU support
pip install ultralytics torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

**Note:** This installation can take 5-10 minutes depending on your internet connection, as it downloads PyTorch and other ML libraries.

## Testing Vision System

### 1. Verify Installation

```bash
python -c "from ultralytics import YOLO; print('YOLO installed successfully')"
```

### 2. Download YOLO Model

The first time you run detection, YOLO will automatically download its model weights:

```bash
python -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); print('Model ready')"
```

This downloads the YOLOv8 nano model (~6MB).

### 3. Run Vision Demo

```bash
# Basic vision test (no GUI)
python scripts/vision_demo.py --vision

# With visualization
python scripts/vision_demo.py --gui --vision
```

## Vision System Components

### Object Detector (`drone_core.vision.detector`)
- Supports multiple backends: YOLO, ONNX, OpenCV DNN
- Configurable confidence thresholds
- COCO dataset (80 classes)
- Real-time inference

### Object Tracker (`drone_core.vision.tracker`)
- Multi-object tracking
- Track ID management
- Lost track recovery
- Visualization utilities

### Vision System (`drone_core.vision.system`)
- Integrated detection + tracking pipeline
- Threat classification
- FPS monitoring
- Annotated video output

## Model Options

### YOLOv8 Models

| Model | Size | Speed | Accuracy | Use Case |
|-------|------|-------|----------|----------|
| yolov8n.pt | 6MB | ~40 FPS (CPU) | Good | Real-time, embedded |
| yolov8s.pt | 22MB | ~30 FPS (CPU) | Better | Balanced |
| yolov8m.pt | 52MB | ~15 FPS (CPU) | Great | Accuracy focused |
| yolov8l.pt | 87MB | ~10 FPS (CPU) | Excellent | Offline analysis |

**Recommendation:** Start with `yolov8n` for development, upgrade to `yolov8m` for production.

### Changing Models

Edit `scripts/vision_demo.py` or your mission config:

```python
vision_config = {
    "detector": {
        "model_type": "yolo",
        "model_size": "n",  # Change to: s, m, l, x
        "confidence_threshold": 0.5
    }
}
```

## GPU Acceleration

### Check GPU Availability

```bash
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### Enable GPU

No code changes needed - PyTorch automatically uses GPU if available.

Expected speedup: 3-5x faster inference

## Troubleshooting

### ImportError: No module named 'ultralytics'

```bash
# Make sure you're in the virtual environment
source venv/bin/activate

# Install ultralytics
pip install ultralytics
```

### Model Download Fails

```bash
# Manual download
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# Place in: ~/.cache/ultralytics/ or specify path in config
```

### Slow Performance

1. **Use smaller model:** Switch from `yolov8m` to `yolov8n`
2. **Enable GPU:** Install CUDA-enabled PyTorch
3. **Reduce resolution:** Lower camera resolution in config
4. **Skip frames:** Process every Nth frame instead of all

### Out of Memory

```bash
# Use nano model
pip install ultralytics

# In your config, use:
model_size = "n"

# Or increase system swap
```

## Performance Benchmarks

### CPU (Intel i7-10th gen)
- YOLOv8n: ~35 FPS
- YOLOv8s: ~25 FPS
- YOLOv8m: ~12 FPS

### GPU (NVIDIA GTX 1080)
- YOLOv8n: ~120 FPS
- YOLOv8s: ~90 FPS
- YOLOv8m: ~60 FPS

### Raspberry Pi 4 (4GB)
- YOLOv8n: ~5-8 FPS
- Not recommended: YOLOv8s+

### Jetson Nano
- YOLOv8n: ~15-20 FPS
- YOLOv8s: ~10-12 FPS

## Configuration Examples

### High Accuracy (Offline Processing)

```python
vision_config = {
    "detector": {
        "model_type": "yolo",
        "model_size": "l",
        "confidence_threshold": 0.7
    },
    "tracker": {
        "max_distance": 100,
        "max_disappeared": 60,
        "min_hits": 5
    }
}
```

### Real-time (Embedded)

```python
vision_config = {
    "detector": {
        "model_type": "yolo",
        "model_size": "n",
        "confidence_threshold": 0.4
    },
    "tracker": {
        "max_distance": 50,
        "max_disappeared": 15,
        "min_hits": 2
    }
}
```

### Balanced (Default)

```python
vision_config = {
    "detector": {
        "model_type": "yolo",
        "model_size": "s",
        "confidence_threshold": 0.5
    },
    "tracker": {
        "max_distance": 75,
        "max_disappeared": 30,
        "min_hits": 3
    }
}
```

## Docker Setup

To use vision in Docker, add to `Dockerfile`:

```dockerfile
RUN pip install ultralytics torch torchvision
```

Or use separate requirements file:

```bash
# requirements-vision.txt
ultralytics>=8.0.0
torch>=2.0.0
torchvision>=0.15.0
```

## Next Steps

1. Run vision demo: `python scripts/vision_demo.py --vision`
2. Test different models and configurations
3. Integrate vision into your custom missions
4. Deploy to hardware with GPU acceleration

## Support

For issues:
- Check [NEXT_STEPS.md](NEXT_STEPS.md) troubleshooting section
- Review [scripts/README_DEMOS.md](scripts/README_DEMOS.md)
- See ultralytics docs: https://docs.ultralytics.com/

---

**Optional Feature:** Vision capabilities are optional. The drone-core simulation works perfectly without ultralytics installed.
