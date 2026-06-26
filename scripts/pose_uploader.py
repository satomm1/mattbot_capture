#!/usr/bin/env python3
"""Upload sealed pose SQLite chunks to central ingest service."""

from __future__ import annotations

import os
import sys
import time

import rospy
import requests

from capture_utils import (
    find_ready_pose_chunks,
    ingest_health_url,
    ingest_pose_upload_url,
    load_pose_chunk_meta,
    pose_chunk_paths,
    remove_pose_chunk,
)
from dds_utils import RobotIdError, require_robot_id_int


class PoseUploader:
    def __init__(self):
        rospy.init_node("pose_uploader", anonymous=False)

        try:
            self.robot_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        self.pose_spool_dir = rospy.get_param("~pose_spool_dir", "/workspace/catkin_ws/data/pose_spool")
        ingest_ip = rospy.get_param("~ingest_ip", "192.168.50.2")
        ingest_port = int(rospy.get_param("~ingest_port", 8080))
        ingest_scheme = rospy.get_param("~ingest_scheme", "http")
        self.ingest_url = ingest_pose_upload_url(ingest_ip, ingest_port, ingest_scheme)
        self.health_url = ingest_health_url(ingest_ip, ingest_port, ingest_scheme)
        self.api_key = rospy.get_param("~api_key", "")
        self.poll_interval_s = float(rospy.get_param("~poll_interval_s", 30.0))
        self.request_timeout_s = float(rospy.get_param("~request_timeout_s", 120.0))
        self.retry_max_s = float(rospy.get_param("~retry_max_s", 300.0))
        self.check_health = bool(rospy.get_param("~check_health", True))

        self._backoff_s = self.poll_interval_s

        rospy.loginfo(
            "pose_uploader ready robot_id=%s ingest=%s spool=%s",
            self.robot_id,
            self.ingest_url,
            self.pose_spool_dir,
        )

    def _headers(self) -> dict:
        headers = {"X-Robot-Id": str(self.robot_id)}
        if self.api_key:
            headers["X-Api-Key"] = self.api_key
        return headers

    def _central_reachable(self) -> bool:
        if not self.check_health:
            return True
        try:
            resp = requests.get(self.health_url, timeout=5.0)
            return resp.status_code == 200
        except requests.RequestException:
            return False

    def _upload_chunk(self, chunk_id: str) -> bool:
        meta_path, sqlite_path = pose_chunk_paths(self.pose_spool_dir, self.robot_id, chunk_id)
        try:
            meta = load_pose_chunk_meta(self.pose_spool_dir, self.robot_id, chunk_id)
        except (OSError, ValueError) as exc:
            rospy.logerr("skip chunk %s: bad meta: %s", chunk_id, exc)
            return False

        if meta.get("status") != "ready_for_upload":
            return False
        if not os.path.isfile(sqlite_path):
            rospy.logerr("skip chunk %s: missing sqlite", chunk_id)
            return False

        opened = []
        resp = None
        try:
            meta_handle = open(meta_path, "rb")
            chunk_handle = open(sqlite_path, "rb")
            opened = [meta_handle, chunk_handle]
            resp = requests.post(
                self.ingest_url,
                headers=self._headers(),
                files=[
                    ("meta", (f"{chunk_id}.meta.json", meta_handle, "application/json")),
                    ("chunk", (f"{chunk_id}.sqlite", chunk_handle, "application/x-sqlite3")),
                ],
                timeout=self.request_timeout_s,
            )
        except requests.RequestException as exc:
            rospy.logwarn("pose upload failed chunk=%s: %s", chunk_id, exc)
            return False
        finally:
            for handle in opened:
                handle.close()

        if resp is None or resp.status_code not in (200, 201):
            status = resp.status_code if resp is not None else "none"
            body = resp.text[:500] if resp is not None else ""
            rospy.logwarn("pose upload rejected chunk=%s status=%s body=%s", chunk_id, status, body)
            return False

        remove_pose_chunk(self.pose_spool_dir, self.robot_id, chunk_id)
        rospy.loginfo(
            "uploaded pose chunk=%s (%d rows)",
            chunk_id,
            meta.get("row_count", 0),
        )
        return True

    def spin(self):
        while not rospy.is_shutdown():
            if not self._central_reachable():
                rospy.logwarn_throttle(60.0, "central ingest unreachable; will retry")
                time.sleep(self._backoff_s)
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)
                continue

            self._backoff_s = self.poll_interval_s
            chunks = find_ready_pose_chunks(self.pose_spool_dir, self.robot_id)
            if not chunks:
                time.sleep(self.poll_interval_s)
                continue

            for chunk_id in chunks:
                if rospy.is_shutdown():
                    break
                if self._upload_chunk(chunk_id):
                    continue
                time.sleep(min(self._backoff_s, self.poll_interval_s))
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)

            time.sleep(self.poll_interval_s)


if __name__ == "__main__":
    PoseUploader().spin()
