# Contributing to Drone Core

Thank you for your interest in contributing to Drone Core! This document provides guidelines for contributing to the project.

## Development Setup

1. Fork the repository
2. Clone your fork:
```bash
git clone https://github.com/YOUR_USERNAME/drone-core.git
cd drone-core
```

3. Create a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```

4. Install in development mode:
```bash
pip install -r requirements.txt
pip install -e ".[dev]"
```

5. Create a feature branch:
```bash
git checkout -b feature/your-feature-name
```

## Code Standards

### Python Style
- Follow PEP 8 style guide
- Use Black for code formatting: `black src/`
- Use isort for import sorting: `isort src/`
- Type hints are encouraged

### Documentation
- Docstrings for all public functions/classes
- Use Google-style docstrings
- Update relevant docs in `docs/` directory

### Testing
- Write tests for new features
- Maintain or improve code coverage
- Run tests before submitting: `pytest`

## Pull Request Process

1. **Update Documentation**: Ensure docs reflect your changes
2. **Add Tests**: Include tests for new functionality
3. **Run Checks**:
```bash
# Format code
black src/ tests/
isort src/ tests/

# Run linter
flake8 src/ tests/

# Run tests
pytest
```

4. **Commit Messages**: Use clear, descriptive commit messages
   - Format: `type: description`
   - Types: feat, fix, docs, test, refactor, chore
   - Example: `feat: add spiral search pattern to path planner`

5. **Create Pull Request**:
   - Provide clear description of changes
   - Reference any related issues
   - Include screenshots/videos for UI changes

## Project Structure

```
drone-core/
├── src/drone_core/       # Main package
│   ├── hal/              # Hardware abstraction
│   ├── vision/           # Computer vision
│   ├── navigation/       # Path planning
│   ├── missions/         # Mission types
│   └── cli.py            # CLI interface
├── tests/                # Test suite
├── scripts/              # Utility scripts
├── config/               # Configuration files
├── docs/                 # Documentation
└── docker/               # Docker configs
```

## Areas for Contribution

### High Priority
- [ ] Additional mission types
- [ ] ROS2 integration
- [ ] Advanced obstacle avoidance
- [ ] Web-based mission control
- [ ] Multi-drone coordination

### Computer Vision
- [ ] Additional ML model support (TensorRT, etc.)
- [ ] Improved tracking algorithms (DeepSORT, ByteTrack)
- [ ] Optical flow integration
- [ ] Custom object detection models

### Navigation
- [ ] SLAM integration
- [ ] GPS-denied navigation
- [ ] Dynamic replanning
- [ ] Terrain following

### Documentation
- [ ] Tutorial videos
- [ ] More mission examples
- [ ] Hardware setup guides
- [ ] Performance optimization guide

### Testing
- [ ] Unit test coverage
- [ ] Integration tests
- [ ] Hardware-in-the-loop tests
- [ ] Simulation scenario library

## Questions?

Feel free to:
- Open an issue for discussion
- Ask in pull request comments
- Contact maintainers

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
