"""
Area Surveillance Mission

Systematic area search and surveillance mission.
"""

import asyncio
import time
from typing import List, Tuple
from drone_core.missions.base import Mission, MissionResult, MissionStatus


class AreaSurveillance(Mission):
    """
    Area surveillance mission.

    Systematically searches an area using grid or spiral pattern,
    detecting and tracking threats.
    """

    def __init__(self, zone: List[Tuple[float, float]],
                 altitude: float = 15.0,
                 speed: float = 5.0,
                 pattern: str = "grid",
                 spacing: float = 10.0,
                 threat_detection: bool = True,
                 name: str = "Area Surveillance"):
        """
        Initialize area surveillance mission.

        Args:
            zone: List of (x, y) boundary points
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            pattern: Search pattern ('grid' or 'spiral')
            spacing: Grid line spacing in meters
            threat_detection: Enable threat detection
            name: Mission name
        """
        config = {
            "zone": zone,
            "altitude": altitude,
            "speed": speed,
            "pattern": pattern,
            "spacing": spacing,
            "threat_detection": threat_detection
        }
        super().__init__(name, config)

        self.zone = zone
        self.altitude = altitude
        self.speed = speed
        self.pattern = pattern
        self.spacing = spacing
        self.threat_detection = threat_detection

        self.threats_detected = []
        self.distance_traveled = 0.0
        self.area_coverage = 0.0

    async def execute(self, drone, vision_system, navigation_system) -> MissionResult:
        """Execute area surveillance mission."""
        self._start_time = time.time()
        self.status = MissionStatus.RUNNING

        try:
            # Arm and takeoff
            print(f"[{self.name}] Arming drone...")
            if not await drone.arm():
                return self._create_result(
                    MissionStatus.FAILED,
                    errors=["Failed to arm drone"]
                )

            print(f"[{self.name}] Taking off to {self.altitude}m...")
            if not await drone.takeoff(self.altitude):
                return self._create_result(
                    MissionStatus.FAILED,
                    errors=["Failed to takeoff"]
                )

            # Wait for takeoff
            await drone.wait_for_position(
                (0, 0, self.altitude),
                tolerance=0.5,
                timeout=30.0
            )

            print(f"[{self.name}] Starting area search ({self.pattern} pattern)...")

            # Start vision processing
            if self.threat_detection:
                vision_task = asyncio.create_task(
                    self._vision_monitoring(drone, vision_system)
                )
            else:
                vision_task = None

            # Execute search pattern
            success = await navigation_system.search_area(
                self.zone,
                pattern=self.pattern,
                altitude=self.altitude,
                speed=self.speed,
                spacing=self.spacing
            )

            # Cancel vision monitoring
            if vision_task:
                vision_task.cancel()
                try:
                    await vision_task
                except asyncio.CancelledError:
                    pass

            if not success:
                return self._create_result(
                    MissionStatus.FAILED,
                    errors=["Navigation failed"]
                )

            # Land
            print(f"[{self.name}] Landing...")
            await drone.land()
            await asyncio.sleep(5)

            # Disarm
            await drone.disarm()

            self._end_time = time.time()
            self.status = MissionStatus.COMPLETED

            print(f"[{self.name}] Mission complete!")
            print(f"  - Duration: {self.get_elapsed_time():.1f}s")
            print(f"  - Threats detected: {len(self.threats_detected)}")

            return self._create_result(
                MissionStatus.COMPLETED,
                distance=self.distance_traveled,
                threats=len(self.threats_detected),
                data={
                    "threats": self.threats_detected,
                    "pattern": self.pattern,
                    "area_coverage": self.area_coverage
                }
            )

        except Exception as e:
            self._end_time = time.time()
            self.status = MissionStatus.FAILED
            return self._create_result(
                MissionStatus.FAILED,
                errors=[str(e)]
            )

    async def _vision_monitoring(self, drone, vision_system):
        """Background task for vision-based threat detection."""
        while True:
            try:
                # Get camera frame
                frame_data = await drone.get_camera_frame()
                if frame_data is None:
                    await asyncio.sleep(0.1)
                    continue

                # Process frame
                result = vision_system.process_frame(frame_data.image)

                # Check for threats
                if result["threats"]:
                    for threat in result["threats"]:
                        threat_info = {
                            "timestamp": time.time(),
                            "track_id": threat.track_id,
                            "class": threat.class_name,
                            "position": await drone.get_status()
                        }
                        self.threats_detected.append(threat_info)
                        print(f"[{self.name}] THREAT DETECTED: {threat.class_name} (ID: {threat.track_id})")

                await asyncio.sleep(0.1)

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[{self.name}] Vision error: {e}")
                await asyncio.sleep(1.0)
