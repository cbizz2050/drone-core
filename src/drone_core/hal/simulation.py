"""
Simulation Backend - PyBullet-based drone simulation

Implements the DroneInterface for a physics-based simulation environment.
"""

import asyncio
import time
from typing import Optional, Tuple
import numpy as np
import cv2

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except ImportError:
    PYBULLET_AVAILABLE = False

from drone_core.hal.interface import (
    DroneInterface, DroneStatus, CameraFrame,
    DroneMode, DroneState
)


class SimulationBackend(DroneInterface):
    """
    PyBullet-based simulation backend.

    Provides a physics-based simulation of a quadcopter drone with
    camera, GPS, and IMU sensors.
    """

    def __init__(self, config: dict):
        """
        Initialize simulation backend.

        Args:
            config: Configuration dict with keys:
                - use_gui: bool, whether to show PyBullet GUI
                - timestep: float, simulation timestep
                - start_position: tuple, initial (x, y, z) position
        """
        super().__init__(config)

        if not PYBULLET_AVAILABLE:
            raise ImportError(
                "PyBullet not installed. Install with: pip install pybullet"
            )

        self.use_gui = config.get("use_gui", False)
        self.timestep = config.get("timestep", 1.0 / 240.0)
        self.start_pos = config.get("start_position", [0, 0, 1])

        self.physics_client = None
        self.drone_id = None
        self.plane_id = None

        # Drone state
        self._armed = False
        self._mode = DroneMode.STABILIZE
        self._state = DroneState.DISARMED
        self._battery_voltage = 16.8  # 4S LiPo fully charged
        self._home_position = None

        # Control
        self._target_position = None
        self._target_velocity = np.array([0.0, 0.0, 0.0])

        # Camera
        self._camera_resolution = (640, 480)
        self._camera_fov = 60

    async def connect(self) -> bool:
        """Connect to simulation environment."""
        try:
            # Start PyBullet
            if self.use_gui:
                self.physics_client = p.connect(p.GUI)
            else:
                self.physics_client = p.connect(p.DIRECT)

            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(self.timestep)

            # Load ground plane
            self.plane_id = p.loadURDF("plane.urdf")

            # Create simple quadcopter (using a cube as placeholder)
            # In production, load proper quadcopter URDF
            collision_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[0.1, 0.1, 0.05]
            )
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.1, 0.1, 0.05],
                rgbaColor=[0.2, 0.2, 0.8, 1]
            )

            self.drone_id = p.createMultiBody(
                baseMass=1.5,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=self.start_pos,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
            )

            # Set dynamics
            p.changeDynamics(
                self.drone_id, -1,
                linearDamping=0.04,
                angularDamping=0.04
            )

            self._home_position = self.start_pos
            self._connected = True

            # Start simulation loop
            asyncio.create_task(self._simulation_loop())

            return True

        except Exception as e:
            print(f"Failed to connect to simulation: {e}")
            return False

    async def disconnect(self) -> None:
        """Disconnect from simulation."""
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None
        self._connected = False

    async def arm(self) -> bool:
        """Arm the drone."""
        if not self._connected:
            return False

        self._armed = True
        self._state = DroneState.ARMED
        return True

    async def disarm(self) -> bool:
        """Disarm the drone."""
        if not self._connected:
            return False

        # Only disarm if on ground
        status = await self.get_status()
        if status.position[2] < 0.2:  # Less than 20cm altitude
            self._armed = False
            self._state = DroneState.DISARMED
            return True

        return False

    async def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude."""
        if not self._armed:
            return False

        status = await self.get_status()
        self._target_position = [
            status.position[0],
            status.position[1],
            altitude
        ]
        self._state = DroneState.FLYING
        self._mode = DroneMode.GUIDED

        return True

    async def land(self) -> bool:
        """Land at current position."""
        if not self._connected:
            return False

        status = await self.get_status()
        self._target_position = [
            status.position[0],
            status.position[1],
            0.0
        ]
        self._state = DroneState.LANDING
        self._mode = DroneMode.LAND

        return True

    async def return_to_launch(self) -> bool:
        """Return to launch position."""
        if not self._connected or not self._home_position:
            return False

        self._target_position = self._home_position
        self._mode = DroneMode.RTL
        self._state = DroneState.FLYING

        return True

    async def goto_position(self, x: float, y: float, z: float,
                           yaw: Optional[float] = None) -> bool:
        """Fly to specified position."""
        if not self._armed:
            return False

        self._target_position = [x, y, z]
        self._mode = DroneMode.GUIDED
        self._state = DroneState.FLYING

        return True

    async def set_velocity(self, vx: float, vy: float, vz: float,
                          yaw_rate: Optional[float] = None) -> bool:
        """Set drone velocity."""
        if not self._armed:
            return False

        self._target_velocity = np.array([vx, vy, vz])
        self._target_position = None  # Clear position target
        self._mode = DroneMode.GUIDED

        return True

    async def set_mode(self, mode: DroneMode) -> bool:
        """Set flight mode."""
        self._mode = mode
        return True

    async def get_status(self) -> DroneStatus:
        """Get current drone status."""
        if self.drone_id is None:
            # Return default status if not connected
            return DroneStatus(
                position=(0, 0, 0),
                velocity=(0, 0, 0),
                attitude=(0, 0, 0),
                battery_voltage=0,
                battery_percent=0,
                gps_fix=False,
                gps_satellites=0,
                mode=self._mode,
                state=self._state,
                armed=self._armed
            )

        # Get position and orientation from PyBullet
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)

        # Convert quaternion to euler angles
        euler = p.getEulerFromQuaternion(orn)

        # Simulate battery drain
        self._battery_voltage -= 0.0001
        battery_percent = max(0, (self._battery_voltage - 14.8) / (16.8 - 14.8) * 100)

        return DroneStatus(
            position=pos,
            velocity=vel,
            attitude=euler,
            battery_voltage=self._battery_voltage,
            battery_percent=battery_percent,
            gps_fix=True,
            gps_satellites=12,
            mode=self._mode,
            state=self._state,
            armed=self._armed
        )

    async def get_camera_frame(self, camera_id: int = 0) -> Optional[CameraFrame]:
        """Get simulated camera frame."""
        if self.drone_id is None:
            return None

        # Get drone position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        rot_matrix = p.getMatrixFromQuaternion(orn)

        # Camera offset from drone center
        camera_offset = [0.1, 0, 0]  # 10cm forward

        # Calculate camera position
        camera_pos = [
            pos[0] + camera_offset[0] * rot_matrix[0],
            pos[1] + camera_offset[1] * rot_matrix[3],
            pos[2] + camera_offset[2] * rot_matrix[6]
        ]

        # Camera target (looking forward and slightly down)
        target_distance = 10
        target_pos = [
            camera_pos[0] + target_distance * rot_matrix[0],
            camera_pos[1] + target_distance * rot_matrix[3],
            camera_pos[2] - 2  # Look slightly down
        ]

        # Render camera view
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=[0, 0, 1]
        )

        proj_matrix = p.computeProjectionMatrixFOV(
            fov=self._camera_fov,
            aspect=self._camera_resolution[0] / self._camera_resolution[1],
            nearVal=0.1,
            farVal=100
        )

        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self._camera_resolution[0],
            height=self._camera_resolution[1],
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL if self.use_gui else p.ER_TINY_RENDERER
        )

        # Convert to OpenCV format (BGR)
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array.reshape((height, width, 4))
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGR)

        return CameraFrame(
            image=bgr_array,
            timestamp=time.time(),
            camera_id=camera_id,
            resolution=self._camera_resolution
        )

    async def emergency_stop(self) -> None:
        """Emergency stop - kill all motors."""
        self._armed = False
        self._state = DroneState.EMERGENCY
        self._target_position = None
        self._target_velocity = np.array([0.0, 0.0, 0.0])

        # Apply immediate downward force
        if self.drone_id is not None:
            p.resetBaseVelocity(self.drone_id, [0, 0, 0], [0, 0, 0])

    async def _simulation_loop(self):
        """Main simulation loop - runs physics and control."""
        while self._connected:
            if self._armed and self.drone_id is not None:
                await self._apply_control()

            # Step simulation
            p.stepSimulation()

            # Run at ~240Hz
            await asyncio.sleep(self.timestep)

    async def _apply_control(self):
        """Apply control forces to maintain position or velocity."""
        if self.drone_id is None:
            return

        # Get current state
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        vel, ang_vel = p.getBaseVelocity(self.drone_id)

        current_pos = np.array(pos)
        current_vel = np.array(vel)

        # Simple PD controller
        kp = 10.0  # Position gain
        kd = 5.0   # Velocity gain

        if self._target_position is not None:
            # Position control
            target = np.array(self._target_position)
            pos_error = target - current_pos
            vel_error = -current_vel

            force = kp * pos_error + kd * vel_error

        else:
            # Velocity control
            vel_error = self._target_velocity - current_vel
            force = kd * vel_error

        # Add gravity compensation
        force[2] += 9.81 * 1.5  # mass * g

        # Apply force
        p.applyExternalForce(
            self.drone_id, -1,
            force.tolist(),
            pos,
            p.WORLD_FRAME
        )

        # Simple attitude stabilization (keep level)
        euler = p.getEulerFromQuaternion(orn)
        attitude_correction = np.array([
            -euler[0] * 2.0,  # Roll correction
            -euler[1] * 2.0,  # Pitch correction
            -ang_vel[2] * 0.5  # Yaw damping
        ])

        p.applyExternalTorque(
            self.drone_id, -1,
            attitude_correction.tolist(),
            p.WORLD_FRAME
        )

        # Check if landed
        if self._state == DroneState.LANDING and pos[2] < 0.1:
            self._armed = False
            self._state = DroneState.DISARMED
