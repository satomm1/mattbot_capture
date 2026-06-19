"""Local spool writes: JPEG files and atomic manifest updates."""

from __future__ import annotations

import json
import os
import shutil
import tempfile

from .manifest import (
    STATUS_READY,
    STATUS_RECORDING,
    manifest_path,
    new_frame_record,
    new_manifest,
    session_dir,
    utc_now_iso,
)


class SpoolError(Exception):
    pass


class SpoolFullError(SpoolError):
    pass


def _atomic_write_json(path: str, data: dict) -> None:
    """Write JSON via temp file + rename so readers never see partial data."""
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".manifest_", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")
        os.replace(tmp_path, path)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise


def dir_size_bytes(path: str) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


class SessionWriter:
    """Manage one recording session on disk."""

    def __init__(self, spool_dir: str, robot_id: int, session_id: str, trigger: str):
        self.spool_dir = spool_dir
        self.robot_id = robot_id
        self.session_id = session_id
        self.dir = session_dir(spool_dir, robot_id, session_id)
        self.manifest_file = manifest_path(spool_dir, robot_id, session_id)
        self.manifest = new_manifest(robot_id, session_id, trigger)
        os.makedirs(self.dir, exist_ok=True)
        _atomic_write_json(self.manifest_file, self.manifest)

    def append_frame(
        self,
        jpeg_bytes: bytes,
        filename: str,
        ros_sec: int,
        ros_nsec: int,
        pose: dict | None,
        detections: list,
        extra: dict | None = None,
    ) -> str:
        if self.manifest.get("status") != STATUS_RECORDING:
            raise SpoolError("session is not recording")

        image_path = os.path.join(self.dir, filename)
        with open(image_path, "wb") as handle:
            handle.write(jpeg_bytes)

        frame = new_frame_record(filename, ros_sec, ros_nsec, pose, detections, extra)
        self.manifest["frames"].append(frame)
        _atomic_write_json(self.manifest_file, self.manifest)
        return frame["frame_id"]

    def finalize(self) -> None:
        """Mark session ready for uploader."""
        self.manifest["ended_at"] = utc_now_iso()
        self.manifest["status"] = STATUS_READY
        _atomic_write_json(self.manifest_file, self.manifest)


def load_manifest(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def find_ready_sessions(spool_dir: str, robot_id: int) -> list[str]:
    """Return session_ids under robot spool with status ready_for_upload."""
    root = os.path.join(spool_dir, f"robot_{robot_id}")
    if not os.path.isdir(root):
        return []

    ready = []
    for name in os.listdir(root):
        manifest_file = os.path.join(root, name, "manifest.json")
        if not os.path.isfile(manifest_file):
            continue
        try:
            manifest = load_manifest(manifest_file)
        except (OSError, json.JSONDecodeError):
            continue
        if manifest.get("status") == STATUS_READY:
            ready.append(name)
    ready.sort()
    return ready


def remove_session(spool_dir: str, robot_id: int, session_id: str) -> None:
    path = session_dir(spool_dir, robot_id, session_id)
    if os.path.isdir(path):
        shutil.rmtree(path)
