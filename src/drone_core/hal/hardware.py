"""
Hardware Backend - Real drone hardware interface

Implements the DroneInterface for real hardware using MAVLink protocol
to communicate with PX4/ArduPilot flight controllers.
"""

import asyncio
import time
from typing import Optional, Tuple
import numpy as np
import cv2

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

from drone_core.hal.interface import (
    DroneInterface, DroneStatus, CameraFrame,
    DroneMode, DroneState
)


class HardwareBackend(DroneInterface):
    """
    MAVLink-based hardware backend for PX4/ArduPilot flight controllers.

    Connects to a real drone via serial, UDP, or TCP and communicates
    using the MAVLink protocol.
    """

    def __init__(self, config: dict):
        """
        Initialize hardware backend.

        Args:
            config: Configuration dict with keys:
                - connection_string: MAVLink connection (e.g., '/dev/ttyUSB0', 'udp:127.0.0.1:14550')
                - baud_rate: Serial baud rate (default 57600)
                - camera_device: Camera device ID or path
                - home_position: Optional home position override
        """
        super().__init__(config)

        if not MAVLINK_AVAILABLE:
            raise ImportError(
                "PyMAVLink not installed. Install with: pip install pymavlink"
            )

        self.connection_string = config.get("connection_string", "udp:127.0.0.1:14550")
        self.baud_rate = config.get("baud_rate", 57600)
        self.camera_device = config.get("camera_device", 0)

        self.mav_connection = None
        self.camera = None
        self._last_heartbeat = 0
        self._home_position = None

        # Cached state
        self._position = (0, 0, 0)
        self._velocity = (0, 0, 0)
        self._attitude = (0, 0, 0)
        self._battery_voltage = 0
        self._battery_percent = 0
        self._gps_fix = False
        self._gps_satellites = 0
        self._armed = False
        self._mode = DroneMode.STABILIZE
        self._state = DroneState.DISARMED

    async def connect(self) -> bool:
        """Connect to hardware via MAVLink."""
        try:
            # Connect to flight controller
            self.mav_connection = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud_rate
            )

            # Wait for heartbeat
            print("Waiting for heartbeat...")
            self.mav_connection.wait_heartbeat(timeout=10)
            print(f"Connected to system {self.mav_connection.target_system}")

            # Initialize camera
            try:
                self.camera = cv2.VideoCapture(self.camera_device)
                if not self.camera.isOpened():
                    print(f"Warning: Could not open camera {self.camera_device}")
                    self.camera = None
            except Exception as e:
                print(f"Warning: Camera initialization failed: {e}")
                self.camera = None

            self._connected = True

            # Start telemetry monitoring loop
            asyncio.create_task(self._telemetry_loop())

            return True

        except Exception as e:
            print(f"Failed to connect to hardware: {e}")
            return False

    async def disconnect(self) -> None:
        """Disconnect from hardware."""
        if self.camera is not None:
            self.camera.release()
            self.camera = None

        if self.mav_connection is not None:
            self.mav_connection.close()
            self.mav_connection = None

        self._connected = False

    async def arm(self) -> bool:
        """Arm the drone motors."""
        if not self._connected:
            return False

        # Send arm command
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )

        # Wait for acknowledgment
        await asyncio.sleep(0.5)
        return self._armed

    async def disarm(self) -> bool:
        """Disarm the drone motors."""
        if not self._connected:
            return False

        # Send disarm command
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        await asyncio.sleep(0.5)
        return not self._armed

    async def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude."""
        if not self._armed:
            return False

        # Send takeoff command
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

        return True

    async def land(self) -> bool:
        """Land at current position."""
        if not self._connected:
            return False

        # Send land command
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        return True

    async def return_to_launch(self) -> bool:
        """Return to launch position."""
        if not self._connected:
            return False

        # Set RTL mode
        return await self.set_mode(DroneMode.RTL)

    async def goto_position(self, x: float, y: float, z: float,
                           yaw: Optional[float] = None) -> bool:
        """Fly to specified position (local NED coordinates)."""
        if not self._armed:
            return False

        # Send position target
        yaw_angle = yaw if yaw is not None else 0

        self.mav_connection.mav.set_position_target_local_ned_send(
            0,  # timestamp
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Position only
            x, y, z,  # Position
            0, 0, 0,  # Velocity (ignored)
            0, 0, 0,  # Acceleration (ignored)
            yaw_angle, 0  # Yaw, yaw rate
        )

        return True

    async def set_velocity(self, vx: float, vy: float, vz: float,
                          yaw_rate: Optional[float] = None) -> bool:
        """Set drone velocity."""
        if not self._armed:
            return False

        yaw_r = yaw_rate if yaw_rate is not None else 0

        self.mav_connection.mav.set_position_target_local_ned_send(
            0,
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # Velocity only
            0, 0, 0,  # Position (ignored)
            vx, vy, vz,  # Velocity
            0, 0, 0,  # Acceleration (ignored)
            0, yaw_r  # Yaw (ignored), yaw rate
        )

        return True

    async def set_mode(self, mode: DroneMode) -> bool:
        """Set flight mode."""
        if not self._connected:
            return False

        # Map our modes to MAVLink modes (PX4)
        mode_map = {
            DroneMode.MANUAL: "MANUAL",
            DroneMode.STABILIZE: "STABILIZED",
            DroneMode.GUIDED: "OFFBOARD",
            DroneMode.AUTO: "AUTO.MISSION",
            DroneMode.LOITER: "AUTO.LOITER",
            DroneMode.RTL: "AUTO.RTL",
            DroneMode.LAND: "AUTO.LAND",
        }

        mode_name = mode_map.get(mode, "STABILIZED")

        # Get mode ID
        if not hasattr(self.mav_connection.target_system, 'mode_mapping'):
            print(f"Warning: Could not get mode mapping, using mode name: {mode_name}")
            return False

        mode_id = self.mav_connection.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"Warning: Unknown mode {mode_name}")
            return False

        # Send mode change command
        self.mav_connection.set_mode(mode_id)
        self._mode = mode

        return True

    async def get_status(self) -> DroneStatus:
        """Get current drone status from cached telemetry."""
        return DroneStatus(
            position=self._position,
            velocity=self._velocity,
            attitude=self._attitude,
            battery_voltage=self._battery_voltage,
            battery_percent=self._battery_percent,
            gps_fix=self._gps_fix,
            gps_satellites=self._gps_satellites,
            mode=self._mode,
            state=self._state,
            armed=self._armed
        )

    async def get_camera_frame(self, camera_id: int = 0) -> Optional[CameraFrame]:
        """Get camera frame from USB/CSI camera."""
        if self.camera is None or not self.camera.isOpened():
            return None

        ret, frame = self.camera.read()
        if not ret:
            return None

        height, width = frame.shape[:2]

        return CameraFrame(
            image=frame,
            timestamp=time.time(),
            camera_id=camera_id,
            resolution=(width, height)
        )

    async def emergency_stop(self) -> None:
        """Emergency stop - disarm immediately."""
        if self.mav_connection is not None:
            # Force disarm
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0  # Force disarm
            )

    async def _telemetry_loop(self):
        """Background loop to receive and process telemetry."""
        while self._connected:
            try:
                # Receive message (non-blocking)
                msg = self.mav_connection.recv_match(blocking=False, timeout=0.01)

                if msg is not None:
                    await self._process_message(msg)

                await asyncio.sleep(0.01)

            except Exception as e:
                print(f"Telemetry error: {e}")
                await asyncio.sleep(0.1)

    async def _process_message(self, msg):
        """Process incoming MAVLink message."""
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            self._last_heartbeat = time.time()
            self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

        elif msg_type == "LOCAL_POSITION_NED":
            self._position = (msg.x, msg.y, msg.z)
            self._velocity = (msg.vx, msg.vy, msg.vz)

        elif msg_type == "ATTITUDE":
            self._attitude = (msg.roll, msg.pitch, msg.yaw)

        elif msg_type == "BATTERY_STATUS":
            self._battery_voltage = msg.voltages[0] / 1000.0  # mV to V
            self._battery_percent = msg.battery_remaining

        elif msg_type == "GPS_RAW_INT":
            self._gps_fix = msg.fix_type >= 3
            self._gps_satellites = msg.satellites_visible

            # Store home position on first GPS fix
            if self._home_position is None and self._gps_fix:
                self._home_position = (
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000.0
                )
