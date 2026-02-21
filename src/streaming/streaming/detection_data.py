"""
Data classes for structured detection data handling.
"""
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class BoundingBox:
    """Represents a single bounding box with two corner points."""
    x1: int
    y1: int
    x2: int
    y2: int

    def to_tuple(self) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """Return as tuple of corner points: ((x1, y1), (x2, y2))"""
        return ((self.x1, self.y1), (x2, self.y2))

    def to_list(self) -> List[int]:
        """Return as flattened list: [x1, y1, x2, y2]"""
        return [self.x1, self.y1, self.x2, self.y2]

    @classmethod
    def from_list(cls, coords: List[int]) -> "BoundingBox":
        """Create from flattened list [x1, y1, x2, y2]"""
        if len(coords) != 4:
            raise ValueError(f"Expected 4 coordinates, got {len(coords)}")
        return cls(x1=coords[0], y1=coords[1], x2=coords[2], y2=coords[3])

    @classmethod
    def from_tuple(cls, points: Tuple[Tuple[int, int], Tuple[int, int]]) -> "BoundingBox":
        """Create from tuple of corner points: ((x1, y1), (x2, y2))"""
        return cls(x1=points[0][0], y1=points[0][1], x2=points[1][0], y2=points[1][1])


@dataclass
class DetectionData:
    """
    Structured data class for object detection information.
    
    This class encapsulates all detection-related data in a type-safe,
    structured format for transmission over WebRTC data channels.
    """
    image_data: Optional[str] = None  # Base64 encoded image or image bytes
    color_detection: str = ""  # Detected color (e.g., "red", "blue", "green")
    bounding_boxes: List[BoundingBox] = field(default_factory=list)  # List of bounding boxes
    confidence_level: int = 0  # Confidence level (0-100)

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "image_data": self.image_data,
            "color_detection": self.color_detection,
            "bounding_box": [bbox.to_list() for bbox in self.bounding_boxes],
            "confidence_level": self.confidence_level
        }

    def to_json(self) -> str:
        """Convert to JSON string for transmission."""
        import json
        return json.dumps(self.to_dict())

    @classmethod
    def from_dict(cls, data: dict) -> "DetectionData":
        """Create from dictionary (e.g., parsed JSON)."""
        bounding_boxes = []
        bounding_box_data = data.get("bounding_box", [])
        
        # Handle both single bounding box and list of bounding boxes
        if bounding_box_data:
            # Check if it's a single bounding box (flat list of 4)
            if isinstance(bounding_box_data[0], int):
                bounding_boxes.append(BoundingBox.from_list(bounding_box_data))
            else:
                # It's a list of bounding boxes
                for box_coords in bounding_box_data:
                    if isinstance(box_coords, (list, tuple)) and len(box_coords) == 4:
                        bounding_boxes.append(BoundingBox.from_list(list(box_coords)))

        return cls(
            image_data=data.get("image_data"),
            color_detection=data.get("color_detection", ""),
            bounding_boxes=bounding_boxes,
            confidence_level=data.get("confidence_level", 0)
        )

    @classmethod
    def from_json(cls, json_str: str) -> "DetectionData":
        """Create from JSON string."""
        import json
        return cls.from_dict(json.loads(json_str))

    def __str__(self) -> str:
        """String representation for debugging."""
        return (
            f"DetectionData(color='{self.color_detection}', "
            f"boxes={len(self.bounding_boxes)}, confidence={self.confidence_level})"
        )
