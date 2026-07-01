"""Serialize DetectedObjectArray messages for capture manifest and detection spool."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from mattbot_image_detection.msg import DetectedObjectArray


def detected_objects_to_list(
    msg: DetectedObjectArray | None,
    min_confidence: float = 0.0,
) -> list[dict]:
    """Convert DetectedObjectArray to JSON-serializable list (map-frame pose in objects)."""
    if msg is None:
        return []

    out = []
    for obj in msg.objects:
        if float(obj.probability) < min_confidence:
            continue
        out.append(
            {
                "class_name": obj.class_name,
                "probability": float(obj.probability),
                "pose": {
                    "x": obj.pose.position.x,
                    "y": obj.pose.position.y,
                    "z": obj.pose.position.z,
                },
                "width": float(obj.width),
                "bbox": [float(obj.x1), float(obj.y1), float(obj.x2), float(obj.y2)],
            }
        )
    return out
