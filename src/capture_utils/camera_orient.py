"""Orient saved camera frames to match OSOD when the camera is upside-down (tall robot)."""

from __future__ import annotations

import cv2
import numpy as np


def orient_bgr(image, tall: bool):
    """RGB/BGR: 180° rotation when tall (same as detect_with_dist_osod.py)."""
    if tall:
        return cv2.rotate(image, cv2.ROTATE_180)
    return image


def orient_mono8(image, tall: bool):
    """IR mono8: same 180° rotation as RGB."""
    if tall:
        return cv2.rotate(image, cv2.ROTATE_180)
    return image


def orient_depth_u16(depth, tall: bool):
    """Depth 16UC1: flip both axes when tall (same as detect_with_dist_osod.py)."""
    if tall:
        return np.flip(depth)
    return depth
