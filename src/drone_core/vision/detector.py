"""
Object Detection Module

Provides object detection using YOLO or other ML models.
Supports ONNX models for efficient inference.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
import cv2


@dataclass
class Detection:
    """Represents a detected object."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    center: Tuple[int, int]  # (cx, cy)
    area: int


class ObjectDetector:
    """
    Object detection using ML models.

    Supports multiple backends:
    - YOLO via ultralytics
    - ONNX runtime
    - OpenCV DNN
    """

    def __init__(self, config: dict):
        """
        Initialize object detector.

        Args:
            config: Configuration dict with keys:
                - model_type: 'yolo', 'onnx', or 'opencv'
                - model_path: Path to model file
                - confidence_threshold: Detection confidence threshold
                - nms_threshold: Non-max suppression threshold
                - input_size: Model input size (width, height)
                - classes: List of class names
        """
        self.config = config
        self.model_type = config.get("model_type", "yolo")
        self.model_path = config.get("model_path", None)
        self.confidence_threshold = config.get("confidence_threshold", 0.5)
        self.nms_threshold = config.get("nms_threshold", 0.4)
        self.input_size = config.get("input_size", (640, 640))

        # COCO class names by default
        self.classes = config.get("classes", self._get_coco_classes())

        self.model = None
        self._load_model()

    def _get_coco_classes(self) -> List[str]:
        """Get COCO dataset class names."""
        return [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def _load_model(self):
        """Load the detection model."""
        if self.model_type == "yolo":
            self._load_yolo_model()
        elif self.model_type == "onnx":
            self._load_onnx_model()
        elif self.model_type == "opencv":
            self._load_opencv_model()
        else:
            print(f"Warning: Unknown model type '{self.model_type}', using dummy detector")
            self.model = None

    def _load_yolo_model(self):
        """Load YOLO model using ultralytics."""
        try:
            from ultralytics import YOLO
            model_path = self.model_path or "yolov8n.pt"
            self.model = YOLO(model_path)
            print(f"Loaded YOLO model from {model_path}")
        except ImportError:
            print("Warning: ultralytics not installed. Install with: pip install ultralytics")
            self.model = None
        except Exception as e:
            print(f"Warning: Failed to load YOLO model: {e}")
            self.model = None

    def _load_onnx_model(self):
        """Load ONNX model using ONNX Runtime."""
        try:
            import onnxruntime as ort
            if self.model_path is None:
                print("Error: model_path required for ONNX backend")
                self.model = None
                return

            self.model = ort.InferenceSession(
                self.model_path,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )
            print(f"Loaded ONNX model from {self.model_path}")
        except ImportError:
            print("Warning: onnxruntime not installed. Install with: pip install onnxruntime")
            self.model = None
        except Exception as e:
            print(f"Warning: Failed to load ONNX model: {e}")
            self.model = None

    def _load_opencv_model(self):
        """Load model using OpenCV DNN."""
        try:
            if self.model_path is None:
                print("Error: model_path required for OpenCV backend")
                self.model = None
                return

            self.model = cv2.dnn.readNet(self.model_path)
            print(f"Loaded OpenCV model from {self.model_path}")
        except Exception as e:
            print(f"Warning: Failed to load OpenCV model: {e}")
            self.model = None

    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Detect objects in a frame.

        Args:
            frame: Input image (BGR format)

        Returns:
            List of Detection objects
        """
        if self.model is None:
            # Return dummy detections for testing without a model
            return self._dummy_detect(frame)

        if self.model_type == "yolo":
            return self._detect_yolo(frame)
        elif self.model_type == "onnx":
            return self._detect_onnx(frame)
        elif self.model_type == "opencv":
            return self._detect_opencv(frame)

        return []

    def _detect_yolo(self, frame: np.ndarray) -> List[Detection]:
        """Detect using YOLO model."""
        if self.model is None:
            return []

        # Run inference
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)

        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])

                # Calculate center and area
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                area = (x2 - x1) * (y2 - y1)

                # Get class name
                class_name = self.classes[class_id] if class_id < len(self.classes) else f"class_{class_id}"

                detection = Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(x1, y1, x2, y2),
                    center=(cx, cy),
                    area=area
                )
                detections.append(detection)

        return detections

    def _detect_onnx(self, frame: np.ndarray) -> List[Detection]:
        """Detect using ONNX model."""
        # Placeholder for ONNX inference
        # Implement based on specific ONNX model architecture
        return []

    def _detect_opencv(self, frame: np.ndarray) -> List[Detection]:
        """Detect using OpenCV DNN."""
        # Placeholder for OpenCV DNN inference
        # Implement based on specific model architecture
        return []

    def _dummy_detect(self, frame: np.ndarray) -> List[Detection]:
        """
        Dummy detector for testing without a model.

        Simulates random detections for development/testing.
        """
        # Return empty list in real scenario, or add test detections
        return []

    def draw_detections(self, frame: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """
        Draw detections on frame.

        Args:
            frame: Input image
            detections: List of detections to draw

        Returns:
            Frame with drawn detections
        """
        output = frame.copy()

        for det in detections:
            x1, y1, x2, y2 = det.bbox

            # Draw bounding box
            color = (0, 255, 0)  # Green for normal objects
            if det.class_name == "person":
                color = (0, 0, 255)  # Red for persons (potential threats)

            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            y_label = max(y1, label_size[1] + 10)

            cv2.rectangle(
                output,
                (x1, y_label - label_size[1] - 10),
                (x1 + label_size[0], y_label),
                color,
                -1
            )
            cv2.putText(
                output,
                label,
                (x1, y_label - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

            # Draw center point
            cv2.circle(output, det.center, 5, color, -1)

        return output

    def filter_detections(self, detections: List[Detection],
                         classes: Optional[List[str]] = None,
                         min_confidence: Optional[float] = None,
                         min_area: Optional[int] = None) -> List[Detection]:
        """
        Filter detections by criteria.

        Args:
            detections: List of detections
            classes: Only include these class names
            min_confidence: Minimum confidence threshold
            min_area: Minimum bounding box area

        Returns:
            Filtered list of detections
        """
        filtered = detections

        if classes is not None:
            filtered = [d for d in filtered if d.class_name in classes]

        if min_confidence is not None:
            filtered = [d for d in filtered if d.confidence >= min_confidence]

        if min_area is not None:
            filtered = [d for d in filtered if d.area >= min_area]

        return filtered
