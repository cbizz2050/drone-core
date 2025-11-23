"""Setup configuration for drone-core package."""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="drone-core",
    version="0.1.0",
    author="Cbizz",
    author_email="cbizz2050@users.noreply.github.com",
    description="Autonomous security drone platform with simulation-first development",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/cbizz2050/drone-core",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Hardware",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.10",
    install_requires=[
        "numpy>=1.24.0",
        "opencv-python>=4.8.0",
        "pillow>=10.0.0",
        "pyyaml>=6.0",
        "pymavlink>=2.4.0",
        "pyserial>=3.5",
        "requests>=2.31.0",
        "scipy>=1.11.0",
        "matplotlib>=3.7.0",
    ],
    extras_require={
        "ml": [
            "torch>=2.0.0",
            "torchvision>=0.15.0",
            "onnxruntime>=1.15.0",
            "ultralytics>=8.0.0",  # YOLO
        ],
        "simulation": [
            "pybullet>=3.2.5",
            "gymnasium>=0.29.0",
        ],
        "dev": [
            "pytest>=7.4.0",
            "pytest-cov>=4.1.0",
            "black>=23.7.0",
            "flake8>=6.1.0",
            "mypy>=1.5.0",
            "isort>=5.12.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "drone-sim=drone_core.cli:simulate",
            "drone-mission=drone_core.cli:mission",
            "drone-control=drone_core.cli:control",
        ],
    },
)
