"""Move uploaded spool items to a permanent on-robot archive."""

from __future__ import annotations

import json
import os
import shutil
import tempfile

from .detection_spool import detection_chunk_paths, detection_spool_root
from .manifest import (
    STATUS_UPLOADED,
    manifest_path,
    robot_spool_root,
    session_dir,
    utc_now_iso,
)
from .pose_spool import pose_chunk_paths, pose_spool_root
from .spool import load_manifest


def _atomic_write_json(path: str, data: dict) -> None:
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".meta_", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
            handle.write("\n")
        os.replace(tmp_path, path)
    except Exception:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise


def archive_robot_root(archive_dir: str, robot_id: int) -> str:
    return os.path.join(archive_dir, f"robot_{robot_id}")


def mark_uploaded(meta_path: str, ingest_status: int) -> None:
    with open(meta_path, encoding="utf-8") as handle:
        meta = json.load(handle)
    meta["status"] = STATUS_UPLOADED
    meta["uploaded_at"] = utc_now_iso()
    meta["ingest_status"] = ingest_status
    _atomic_write_json(meta_path, meta)


def archive_session(archive_dir: str, spool_dir: str, robot_id: int, session_id: str) -> bool:
    src = session_dir(spool_dir, robot_id, session_id)
    if not os.path.isdir(src):
        return False
    dest = os.path.join(archive_robot_root(archive_dir, robot_id), "sessions", session_id)
    if os.path.exists(dest):
        return True
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.move(src, dest)
    return True


def archive_pose_chunk(archive_dir: str, spool_dir: str, robot_id: int, chunk_id: str) -> bool:
    meta_path, sqlite_path = pose_chunk_paths(spool_dir, robot_id, chunk_id)
    if not os.path.isfile(meta_path) or not os.path.isfile(sqlite_path):
        return False
    dest_dir = os.path.join(archive_robot_root(archive_dir, robot_id), "pose")
    os.makedirs(dest_dir, exist_ok=True)
    dest_meta = os.path.join(dest_dir, f"{chunk_id}.meta.json")
    dest_sqlite = os.path.join(dest_dir, f"{chunk_id}.sqlite")
    if os.path.exists(dest_meta) and os.path.exists(dest_sqlite):
        return True
    shutil.move(meta_path, dest_meta)
    shutil.move(sqlite_path, dest_sqlite)
    return True


def archive_detection_chunk(
    archive_dir: str, spool_dir: str, robot_id: int, chunk_id: str
) -> bool:
    meta_path, sqlite_path = detection_chunk_paths(spool_dir, robot_id, chunk_id)
    if not os.path.isfile(meta_path) or not os.path.isfile(sqlite_path):
        return False
    dest_dir = os.path.join(archive_robot_root(archive_dir, robot_id), "detection")
    os.makedirs(dest_dir, exist_ok=True)
    dest_meta = os.path.join(dest_dir, f"{chunk_id}.meta.json")
    dest_sqlite = os.path.join(dest_dir, f"{chunk_id}.sqlite")
    if os.path.exists(dest_meta) and os.path.exists(dest_sqlite):
        return True
    shutil.move(meta_path, dest_meta)
    shutil.move(sqlite_path, dest_sqlite)
    return True


def find_pending_archive_sessions(spool_dir: str, robot_id: int) -> list[str]:
    root = robot_spool_root(spool_dir, robot_id)
    if not os.path.isdir(root):
        return []
    pending = []
    for name in os.listdir(root):
        manifest_file = os.path.join(root, name, "manifest.json")
        if not os.path.isfile(manifest_file):
            continue
        try:
            manifest = load_manifest(manifest_file)
        except (OSError, json.JSONDecodeError):
            continue
        if manifest.get("status") == STATUS_UPLOADED:
            pending.append(name)
    pending.sort()
    return pending


def _find_pending_chunks(spool_dir: str, robot_id: int, spool_root_fn) -> list[str]:
    root = spool_root_fn(spool_dir, robot_id)
    if not os.path.isdir(root):
        return []
    pending = []
    for name in os.listdir(root):
        if not name.endswith(".meta.json"):
            continue
        meta_path = os.path.join(root, name)
        try:
            with open(meta_path, encoding="utf-8") as handle:
                meta = json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue
        if meta.get("status") == STATUS_UPLOADED:
            chunk_id = meta.get("chunk_id")
            if chunk_id:
                pending.append(chunk_id)
    pending.sort()
    return pending


def find_pending_archive_pose_chunks(spool_dir: str, robot_id: int) -> list[str]:
    return _find_pending_chunks(spool_dir, robot_id, pose_spool_root)


def find_pending_archive_detection_chunks(spool_dir: str, robot_id: int) -> list[str]:
    return _find_pending_chunks(spool_dir, robot_id, detection_spool_root)
