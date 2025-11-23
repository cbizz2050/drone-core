# Drone Core - Simulation Environment
# Ubuntu-based container with Python 3.10+ and all dependencies

FROM python:3.10-slim

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    cmake \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libglu1-mesa \
    libxcursor1 \
    libxinerama1 \
    libxi6 \
    libxrandr2 \
    libxxf86vm1 \
    mesa-utils \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the entire project
COPY . .

# Install the package in development mode
RUN pip install -e .

# Create directories for outputs
RUN mkdir -p /app/output /app/logs

# Expose any ports if needed (for future web interface)
EXPOSE 8000

# Default command - run simulation
CMD ["python", "scripts/run_sortie.py", "--mission", "perimeter_patrol"]
