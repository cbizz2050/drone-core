"""
Vision System - Integrated computer vision pipeline

Combines object detection and tracking for the security drone.
"""

import time
from typing import List, Optional, Callable
import numpy as np
import cv2

from drone_core.vision.detector import ObjectDetector, Detection
from drone_core.vision.tracker import ObjectTracker, TrackedObject


class VisionSystem:
    """
    Integrated vision system for security drone.

    Combines detection and tracking with threat assessment capabilities.
    """

    def __init__(self, config: dict):
        """
        Initialize vision system.

        Args:
            config: Configuration dict with keys:
                - detector: Detector configuration
                - tracker: Tracker configuration
                - threat_classes: List of class names considered threats
                - alert_callback: Optional callback for threat alerts
        """
        self.config = config

        # Initialize detector and tracker
        detector_config = config.get("detector", {})
        tracker_config = config.get("tracker", {})

        self.detector = ObjectDetector(detector_config)
        self.tracker = ObjectTracker(tracker_config)

        # Threat assessment
        self.threat_classes = config.get("threat_classes", ["person"])
        self.alert_callback = config.get("alert_callback", None)

        # Statistics
        self.frame_count = 0
        self.total_detections = 0
        self.total_threats = 0
        self.fps = 0
        self._last_fps_update = time.time()
        self._frame_times = []

    def process_frame(self, frame: np.ndarray) -> dict:
        """
        Process a single frame through the vision pipeline.

        Args:
            frame: Input image (BGR format)

        Returns:
            Dictionary containing:
                - detections: List of Detection objects
                - tracks: List of TrackedObject objects
                - threats: List of threat TrackedObject objects
                - annotated_frame: Frame with visualizations
                - stats: Processing statistics
        """
        start_time = time.time()

        # Run object detection
        detections = self.detector.detect(frame)

        # Update tracker
        timestamp = time.time()
        tracks = self.tracker.update(detections, timestamp)

        # Identify threats
        threats = self._identify_threats(tracks)

        # Trigger alerts if threats detected
        if threats and self.alert_callback:
            self.alert_callback(threats)

        # Update statistics
        self.frame_count += 1
        self.total_detections += len(detections)
        self.total_threats += len(threats)

        # Calculate FPS
        frame_time = time.time() - start_time
        self._frame_times.append(frame_time)
        if len(self._frame_times) > 30:
            self._frame_times.pop(0)

        if time.time() - self._last_fps_update > 1.0:
            avg_frame_time = np.mean(self._frame_times)
            self.fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0
            self._last_fps_update = time.time()

        # Create annotated frame
        annotated_frame = self._annotate_frame(frame, detections, tracks, threats)

        return {
            "detections": detections,
            "tracks": tracks,
            "threats": threats,
            "annotated_frame": annotated_frame,
            "stats": {
                "frame_count": self.frame_count,
                "fps": self.fps,
                "num_detections": len(detections),
                "num_tracks": len(tracks),
                "num_threats": len(threats),
                "processing_time": frame_time
            }
        }

    def _identify_threats(self, tracks: List[TrackedObject]) -> List[TrackedObject]:
        """
        Identify potential threats from tracked objects.

        Args:
            tracks: List of tracked objects

        Returns:
            List of tracks identified as threats
        """
        threats = []

        for track in tracks:
            # Check if object class is a threat
            if track.class_name in self.threat_classes:
                # Only confirmed tracks are threats
                if track.total_frames >= self.tracker.min_hits:
                    threats.append(track)

        return threats

    def _annotate_frame(self, frame: np.ndarray,
                       detections: List[Detection],
                       tracks: List[TrackedObject],
                       threats: List[TrackedObject]) -> np.ndarray:
        """
        Create annotated frame with all visualizations.

        Args:
            frame: Original frame
            detections: Current detections
            tracks: Current tracks
            threats: Identified threats

        Returns:
            Annotated frame
        """
        output = frame.copy()

        # Draw all tracks
        output = self.tracker.draw_tracks(output, tracks)

        # Highlight threats with red boxes
        for threat in threats:
            det = threat.last_detection
            x1, y1, x2, y2 = det.bbox

            # Draw thick red border for threats
            cv2.rectangle(output, (x1-3, y1-3), (x2+3, y2+3), (0, 0, 255), 3)

            # Add THREAT label
            cv2.putText(
                output, "THREAT",
                (x1, y1 - 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # Add stats overlay
        self._draw_stats_overlay(output, len(detections), len(tracks), len(threats))

        return output

    def _draw_stats_overlay(self, frame: np.ndarray,
                           num_detections: int,
                           num_tracks: int,
                           num_threats: int):
        """Draw statistics overlay on frame."""
        height, width = frame.shape[:2]

        # Create semi-transparent overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, 150), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Draw text
        y_offset = 35
        line_height = 25

        stats = [
            f"FPS: {self.fps:.1f}",
            f"Detections: {num_detections}",
            f"Tracks: {num_tracks}",
            f"Threats: {num_threats}",
        ]

        for i, stat in enumerate(stats):
            color = (0, 255, 0)  # Green
            if i == 3 and num_threats > 0:
                color = (0, 0, 255)  # Red for threats

            cv2.putText(
                frame, stat,
                (20, y_offset + i * line_height),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

    def reset(self):
        """Reset vision system state."""
        self.tracker.reset()
        self.frame_count = 0
        self.total_detections = 0
        self.total_threats = 0
        self._frame_times.clear()

    def get_stats(self) -> dict:
        """Get vision system statistics."""
        return {
            "frame_count": self.frame_count,
            "total_detections": self.total_detections,
            "total_threats": self.total_threats,
            "fps": self.fps,
            "active_tracks": len(self.tracker.tracked_objects),
        }
