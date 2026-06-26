"""Local pose spool: SQLite chunks with hourly rotation."""

from __future__ import annotations

import json
import os
import sqlite3
import threading
from datetime import datetime, timezone

from .manifest import STATUS_READY, STATUS_RECORDING, utc_now_iso

SCHEMA_VERSION = 1
ACTIVE_SQLITE = "active.sqlite"
ACTIVE_META = "active.meta.json"

_CREATE_POSES = """
CREATE TABLE IF NOT EXISTS poses (
  id            INTEGER PRIMARY KEY,
  wall_time     TEXT NOT NULL,
  ros_time_sec  INTEGER,
  ros_time_nsec INTEGER,
  x             REAL, y REAL, theta REAL,
  frame         TEXT,
  ref_x         REAL, ref_y REAL, ref_theta REAL,
  is_static     INTEGER NOT NULL,
  valid         INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_poses_wall_time ON poses(wall_time);
"""

_INSERT = """
INSERT INTO poses (
  wall_time, ros_time_sec, ros_time_nsec,
  x, y, theta, frame, ref_x, ref_y, ref_theta, is_static, valid
) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
"""


class PoseSpoolError(Exception):
    pass


def pose_spool_root(spool_dir: str, robot_id: int) -> str:
    return os.path.join(spool_dir, f"robot_{robot_id}")


def _chunk_id_from_started(started_at: str) -> str:
    return "chunk_" + started_at.replace(":", "-").replace("+00:00", "Z")


def _atomic_write_json(path: str, data: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")
    os.replace(tmp, path)


class PoseChunkWriter:
    """One open SQLite chunk; rotate by close + rename."""

    def __init__(self, spool_dir: str, robot_id: int):
        self.spool_dir = spool_dir
        self.robot_id = robot_id
        self.root = pose_spool_root(spool_dir, robot_id)
        os.makedirs(self.root, exist_ok=True)
        self.started_at = utc_now_iso()
        self.chunk_id = _chunk_id_from_started(self.started_at)
        self.sqlite_path = os.path.join(self.root, ACTIVE_SQLITE)
        self.meta_path = os.path.join(self.root, ACTIVE_META)
        self._lock = threading.Lock()
        self._conn = None
        self._insert = None
        self._row_count = 0
        self._open_db()
        _atomic_write_json(
            self.meta_path,
            {
                "schema_version": SCHEMA_VERSION,
                "robot_id": robot_id,
                "chunk_id": self.chunk_id,
                "started_at": self.started_at,
                "ended_at": None,
                "row_count": 0,
                "status": STATUS_RECORDING,
            },
        )

    def _open_db(self) -> None:
        # check_same_thread=False: rospy.Timer runs inserts on a worker thread
        self._conn = sqlite3.connect(self.sqlite_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.executescript(_CREATE_POSES)
        self._insert = self._conn.cursor()

    def insert(self, row: dict) -> None:
        with self._lock:
            self._insert.execute(
                _INSERT,
                (
                    row["wall_time"],
                    row.get("ros_time_sec"),
                    row.get("ros_time_nsec"),
                    row.get("x"),
                    row.get("y"),
                    row.get("theta"),
                    row.get("frame"),
                    row.get("ref_x"),
                    row.get("ref_y"),
                    row.get("ref_theta"),
                    int(row["is_static"]),
                    int(row["valid"]),
                ),
            )
            self._conn.commit()
            self._row_count += 1

    def maybe_rotate(self, chunk_hours: float) -> bool:
        """Seal active chunk if older than chunk_hours. Returns True if rotated."""
        with self._lock:
            if chunk_hours <= 0:
                return False
            started = datetime.fromisoformat(self.started_at)
            if started.tzinfo is None:
                started = started.replace(tzinfo=timezone.utc)
            age_h = (datetime.now(timezone.utc) - started).total_seconds() / 3600.0
            if age_h < chunk_hours:
                return False
            self._seal_unlocked()
            return True

    def _seal_unlocked(self) -> None:
        ended_at = utc_now_iso()
        self._conn.close()
        self._conn = None
        self._insert = None

        sealed_sqlite = os.path.join(self.root, f"{self.chunk_id}.sqlite")
        sealed_meta = os.path.join(self.root, f"{self.chunk_id}.meta.json")
        os.replace(self.sqlite_path, sealed_sqlite)
        if os.path.isfile(self.meta_path):
            os.remove(self.meta_path)

        _atomic_write_json(
            sealed_meta,
            {
                "schema_version": SCHEMA_VERSION,
                "robot_id": self.robot_id,
                "chunk_id": self.chunk_id,
                "started_at": self.started_at,
                "ended_at": ended_at,
                "row_count": self._row_count,
                "status": STATUS_READY,
            },
        )

    def reopen(self) -> None:
        """Open a fresh active chunk after rotation."""
        with self._lock:
            self.started_at = utc_now_iso()
            self.chunk_id = _chunk_id_from_started(self.started_at)
            self.sqlite_path = os.path.join(self.root, ACTIVE_SQLITE)
            self.meta_path = os.path.join(self.root, ACTIVE_META)
            self._open_db()
            self._row_count = 0
            _atomic_write_json(
                self.meta_path,
                {
                    "schema_version": SCHEMA_VERSION,
                    "robot_id": self.robot_id,
                    "chunk_id": self.chunk_id,
                    "started_at": self.started_at,
                    "ended_at": None,
                    "row_count": 0,
                    "status": STATUS_RECORDING,
                },
            )

    def close(self) -> int:
        """Seal active chunk if it has rows. Returns rows sealed (0 if none)."""
        with self._lock:
            if self._conn is None:
                return 0
            if self._row_count > 0:
                count = self._row_count
                self._seal_unlocked()
                return count
            self._conn.close()
            self._conn = None
            self._insert = None
            for path in (self.sqlite_path, self.meta_path):
                if os.path.isfile(path):
                    os.remove(path)
            return 0


def find_ready_pose_chunks(spool_dir: str, robot_id: int) -> list[str]:
    root = pose_spool_root(spool_dir, robot_id)
    if not os.path.isdir(root):
        return []
    ready = []
    for name in os.listdir(root):
        if not name.endswith(".meta.json") or name == ACTIVE_META:
            continue
        meta_path = os.path.join(root, name)
        try:
            with open(meta_path, encoding="utf-8") as handle:
                meta = json.load(handle)
        except (OSError, json.JSONDecodeError):
            continue
        if meta.get("status") == STATUS_READY:
            chunk_id = meta.get("chunk_id")
            if chunk_id:
                ready.append(chunk_id)
    ready.sort()
    return ready


def pose_chunk_paths(spool_dir: str, robot_id: int, chunk_id: str) -> tuple[str, str]:
    root = pose_spool_root(spool_dir, robot_id)
    return (
        os.path.join(root, f"{chunk_id}.meta.json"),
        os.path.join(root, f"{chunk_id}.sqlite"),
    )


def load_pose_chunk_meta(spool_dir: str, robot_id: int, chunk_id: str) -> dict:
    meta_path, _ = pose_chunk_paths(spool_dir, robot_id, chunk_id)
    with open(meta_path, encoding="utf-8") as handle:
        return json.load(handle)


def remove_pose_chunk(spool_dir: str, robot_id: int, chunk_id: str) -> None:
    meta_path, sqlite_path = pose_chunk_paths(spool_dir, robot_id, chunk_id)
    for path in (meta_path, sqlite_path):
        if os.path.isfile(path):
            os.remove(path)
