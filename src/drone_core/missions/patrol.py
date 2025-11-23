"""
Perimeter Patrol Mission

Autonomous perimeter surveillance mission.
"""

import asyncio
import time
from typing import List, Tuple
from drone_core.missions.base import Mission, MissionResult, MissionStatus


class PerimeterPatrol(Mission):
    """
    Perimeter patrol mission.

    Flies a patrol route around a defined perimeter, using computer vision
    to detect potential threats.
    """

    def __init__(self, waypoints: List[Tuple[float, float]],
                 altitude: float = 10.0,
                 speed: float = 5.0,
                 detect_intruders: bool = True,
                 num_loops: int = 1,
                 name: str = "Perimeter Patrol"):
        """
        Initialize perimeter patrol mission.

        Args:
            waypoints: List of (x, y) perimeter boundary points
            altitude: Flight altitude in meters
            speed: Flight speed in m/s
            detect_intruders: Enable intruder detection
            num_loops: Number of patrol loops
            name: Mission name
        """
        config = {
            "waypoints": waypoints,
            "altitude": altitude,
            "speed": speed,
            "detect_intruders": detect_intruders,
            "num_loops": num_loops
        }
        super().__init__(name, config)

        self.waypoints = waypoints
        self.altitude = altitude
        self.speed = speed
        self.detect_intruders = detect_intruders
        self.num_loops = num_loops

        self.threats_detected = []
        self.distance_traveled = 0.0

    async def execute(self, drone, vision_system, navigation_system) -> MissionResult:
        """Execute perimeter patrol mission."""
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

            # Wait for takeoff complete
            await drone.wait_for_position(
                (0, 0, self.altitude),
                tolerance=0.5,
                timeout=30.0
            )

            print(f"[{self.name}] Starting patrol...")

            # Start vision processing in background
            if self.detect_intruders:
                vision_task = asyncio.create_task(
                    self._vision_monitoring(drone, vision_system)
                )
            else:
                vision_task = None

            # Execute patrol
            for loop in range(self.num_loops):
                print(f"[{self.name}] Patrol loop {loop + 1}/{self.num_loops}")

                if self.should_abort():
                    break

                await self.check_pause()

                # Patrol perimeter
                success = await navigation_system.patrol_perimeter(
                    self.waypoints,
                    self.altitude,
                    self.speed,
                    num_loops=1
                )

                if not success:
                    if vision_task:
                        vision_task.cancel()
                    return self._create_result(
                        MissionStatus.FAILED,
                        errors=["Navigation failed"]
                    )

            # Cancel vision monitoring
            if vision_task:
                vision_task.cancel()
                try:
                    await vision_task
                except asyncio.CancelledError:
                    pass

            # Land
            print(f"[{self.name}] Landing...")
            await drone.land()
            await asyncio.sleep(5)  # Wait for landing

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
                    "loops_completed": self.num_loops
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
