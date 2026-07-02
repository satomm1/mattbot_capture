"""Shared helpers for capture manifest JSON and spool paths."""

from __future__ import annotations

import os
import uuid
from datetime import datetime, timezone


SCHEMA_VERSION = 1
STATUS_RECORDING = "recording"
STATUS_READY = "ready_for_upload"
STATUS_UPLOADED = "uploaded"


def utc_now_iso() -> str:
    """Return current UTC time as ISO-8601 string."""
    return datetime.now(timezone.utc).isoformat()


def new_session_id() -> str:
    return str(uuid.uuid4())


def robot_spool_root(spool_dir: str, robot_id: int) -> str:
    return os.path.join(spool_dir, f"robot_{robot_id}")


def session_dir(spool_dir: str, robot_id: int, session_id: str) -> str:
    return os.path.join(robot_spool_root(spool_dir, robot_id), session_id)


def manifest_path(spool_dir: str, robot_id: int, session_id: str) -> str:
    return os.path.join(session_dir(spool_dir, robot_id, session_id), "manifest.json")


def frame_filename(ros_sec: int, ros_nsec: int) -> str:
    return f"frame_{ros_sec}_{ros_nsec}.jpg"


def ir_frame_filename(ros_sec: int, ros_nsec: int) -> str:
    # _ir.jpg suffix pairs with RGB frame_{sec}_{nsec}.jpg for central ingest
    return f"frame_{ros_sec}_{ros_nsec}_ir.jpg"


def depth_frame_filename(ros_sec: int, ros_nsec: int) -> str:
    # _depth.png suffix pairs with RGB for central ingest (16UC1 mm, lossless)
    return f"frame_{ros_sec}_{ros_nsec}_depth.png"


def frame_id_from_filename(filename: str) -> str:
    # frame_123_456.jpg -> frame_123_456; frame_123_456_depth.png -> frame_123_456_depth
    base = os.path.basename(filename)
    for suffix in (".jpg", ".png"):
        if base.endswith(suffix):
            return base[: -len(suffix)]
    return base


def new_manifest(robot_id: int, session_id: str, trigger: str) -> dict:
    """Build an empty session manifest."""
    return {
        "schema_version": SCHEMA_VERSION,
        "robot_id": robot_id,
        "session_id": session_id,
        "trigger": trigger,
        "started_at": utc_now_iso(),
        "ended_at": None,
        "status": STATUS_RECORDING,
        "frames": [],
    }


def new_frame_record(
    filename: str,
    ros_sec: int,
    ros_nsec: int,
    pose: dict | None,
    detections: list,
    extra: dict | None = None,
) -> dict:
    """Build one frame entry for manifest.frames."""
    return {
        "frame_id": frame_id_from_filename(filename),
        "filename": filename,
        "ros_time": {"sec": ros_sec, "nsec": ros_nsec},
        "wall_time": utc_now_iso(),
        "pose": pose,
        "detections": detections or [],
        "extra": extra or {},
    }
