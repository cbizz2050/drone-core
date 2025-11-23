"""
Object Tracking Module

Tracks detected objects across frames using various tracking algorithms.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import numpy as np
import cv2
from drone_core.vision.detector import Detection


@dataclass
class TrackedObject:
    """Represents a tracked object across frames."""
    track_id: int
    class_name: str
    last_detection: Detection
    positions: List[Tuple[int, int]] = field(default_factory=list)
    frames_since_update: int = 0
    total_frames: int = 0
    first_seen: float = 0
    last_seen: float = 0


class ObjectTracker:
    """
    Multi-object tracker using simple centroid-based tracking.

    For production, consider using more sophisticated algorithms like:
    - DeepSORT
    - ByteTrack
    - FairMOT
    """

    def __init__(self, config: dict):
        """
        Initialize object tracker.

        Args:
            config: Configuration dict with keys:
                - max_distance: Maximum distance for association (pixels)
                - max_disappeared: Maximum frames object can disappear
                - min_hits: Minimum hits before confirmed track
        """
        self.max_distance = config.get("max_distance", 50)
        self.max_disappeared = config.get("max_disappeared", 30)
        self.min_hits = config.get("min_hits", 3)

        self.next_track_id = 0
        self.tracked_objects: Dict[int, TrackedObject] = {}

    def update(self, detections: List[Detection], timestamp: float) -> List[TrackedObject]:
        """
        Update tracker with new detections.

        Args:
            detections: List of detections from current frame
            timestamp: Current timestamp

        Returns:
            List of currently tracked objects
        """
        # If no tracked objects, initialize with detections
        if len(self.tracked_objects) == 0:
            for det in detections:
                self._create_track(det, timestamp)
            return list(self.tracked_objects.values())

        # If no detections, increment disappeared counter
        if len(detections) == 0:
            for track in self.tracked_objects.values():
                track.frames_since_update += 1
            self._remove_stale_tracks()
            return list(self.tracked_objects.values())

        # Get current track centers
        track_ids = list(self.tracked_objects.keys())
        track_centers = np.array([
            self.tracked_objects[tid].last_detection.center
            for tid in track_ids
        ])

        # Get detection centers
        detection_centers = np.array([det.center for det in detections])

        # Compute distance matrix
        distances = self._compute_distances(track_centers, detection_centers)

        # Hungarian algorithm for assignment (simplified greedy approach)
        assignments = self._assign_detections(distances)

        # Update assigned tracks
        for track_idx, det_idx in assignments:
            track_id = track_ids[track_idx]
            detection = detections[det_idx]

            track = self.tracked_objects[track_id]
            track.last_detection = detection
            track.positions.append(detection.center)
            track.frames_since_update = 0
            track.total_frames += 1
            track.last_seen = timestamp

        # Create new tracks for unassigned detections
        assigned_dets = set(det_idx for _, det_idx in assignments)
        for det_idx, det in enumerate(detections):
            if det_idx not in assigned_dets:
                self._create_track(det, timestamp)

        # Increment disappeared counter for unassigned tracks
        assigned_tracks = set(track_idx for track_idx, _ in assignments)
        for track_idx, track_id in enumerate(track_ids):
            if track_idx not in assigned_tracks:
                self.tracked_objects[track_id].frames_since_update += 1

        # Remove stale tracks
        self._remove_stale_tracks()

        return list(self.tracked_objects.values())

    def _compute_distances(self, centers1: np.ndarray, centers2: np.ndarray) -> np.ndarray:
        """
        Compute pairwise distances between two sets of centers.

        Args:
            centers1: Array of shape (N, 2)
            centers2: Array of shape (M, 2)

        Returns:
            Distance matrix of shape (N, M)
        """
        # Euclidean distance
        distances = np.linalg.norm(
            centers1[:, np.newaxis, :] - centers2[np.newaxis, :, :],
            axis=2
        )
        return distances

    def _assign_detections(self, distances: np.ndarray) -> List[Tuple[int, int]]:
        """
        Assign detections to tracks using greedy algorithm.

        Args:
            distances: Distance matrix (tracks x detections)

        Returns:
            List of (track_idx, detection_idx) tuples
        """
        assignments = []

        # Greedy assignment (for production, use Hungarian algorithm)
        used_tracks = set()
        used_detections = set()

        # Flatten and sort by distance
        n_tracks, n_dets = distances.shape
        pairs = []
        for i in range(n_tracks):
            for j in range(n_dets):
                if distances[i, j] < self.max_distance:
                    pairs.append((distances[i, j], i, j))

        pairs.sort()  # Sort by distance

        for dist, track_idx, det_idx in pairs:
            if track_idx not in used_tracks and det_idx not in used_detections:
                assignments.append((track_idx, det_idx))
                used_tracks.add(track_idx)
                used_detections.add(det_idx)

        return assignments

    def _create_track(self, detection: Detection, timestamp: float):
        """Create a new track from detection."""
        track = TrackedObject(
            track_id=self.next_track_id,
            class_name=detection.class_name,
            last_detection=detection,
            positions=[detection.center],
            frames_since_update=0,
            total_frames=1,
            first_seen=timestamp,
            last_seen=timestamp
        )

        self.tracked_objects[self.next_track_id] = track
        self.next_track_id += 1

    def _remove_stale_tracks(self):
        """Remove tracks that haven't been updated recently."""
        to_remove = []

        for track_id, track in self.tracked_objects.items():
            if track.frames_since_update > self.max_disappeared:
                to_remove.append(track_id)

        for track_id in to_remove:
            del self.tracked_objects[track_id]

    def get_confirmed_tracks(self) -> List[TrackedObject]:
        """Get only confirmed tracks (met minimum hit threshold)."""
        return [
            track for track in self.tracked_objects.values()
            if track.total_frames >= self.min_hits
        ]

    def draw_tracks(self, frame: np.ndarray, tracks: Optional[List[TrackedObject]] = None) -> np.ndarray:
        """
        Draw tracked objects on frame.

        Args:
            frame: Input image
            tracks: List of tracks to draw (None = all tracks)

        Returns:
            Frame with drawn tracks
        """
        output = frame.copy()

        if tracks is None:
            tracks = list(self.tracked_objects.values())

        for track in tracks:
            det = track.last_detection
            x1, y1, x2, y2 = det.bbox

            # Choose color based on track confirmation
            if track.total_frames >= self.min_hits:
                color = (0, 255, 0)  # Green for confirmed
            else:
                color = (255, 255, 0)  # Yellow for tentative

            # Draw bounding box
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)

            # Draw track ID and class
            label = f"ID:{track.track_id} {track.class_name}"
            cv2.putText(
                output, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

            # Draw trajectory
            if len(track.positions) > 1:
                points = np.array(track.positions, dtype=np.int32)
                cv2.polylines(output, [points], False, color, 2)

        return output

    def reset(self):
        """Reset tracker state."""
        self.tracked_objects.clear()
        self.next_track_id = 0
