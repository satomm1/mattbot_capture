#!/usr/bin/env python3
"""Upload sealed pose SQLite chunks to central ingest service."""

from __future__ import annotations

import os
import sys

import rospy
import requests

from capture_utils import (
    archive_pose_chunk,
    find_pending_archive_pose_chunks,
    find_ready_pose_chunks,
    ingest_health_url,
    ingest_pose_upload_url,
    load_pose_chunk_meta,
    mark_uploaded,
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
        self.archive_dir = rospy.get_param(
            "~archive_dir", "/workspace/catkin_ws/data/upload_archive"
        )
        self.retain_after_upload = bool(rospy.get_param("~retain_after_upload", True))
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
        self._upload_enabled = True

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "pose_uploader ready robot_id=%s ingest=%s spool=%s archive=%s retain=%s",
            self.robot_id,
            self.ingest_url,
            self.pose_spool_dir,
            self.archive_dir,
            self.retain_after_upload,
        )

    def _on_shutdown(self):
        # Seal-only on logger side; do not start uploads while roslaunch is tearing down.
        self._upload_enabled = False

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

    def _finalize_after_upload(self, chunk_id: str, ingest_status: int) -> bool:
        meta_path, _ = pose_chunk_paths(self.pose_spool_dir, self.robot_id, chunk_id)
        try:
            mark_uploaded(meta_path, ingest_status)
        except OSError as exc:
            rospy.logerr("chunk %s: failed to mark uploaded: %s", chunk_id, exc)
            return False

        if self.retain_after_upload:
            try:
                if not archive_pose_chunk(
                    self.archive_dir, self.pose_spool_dir, self.robot_id, chunk_id
                ):
                    rospy.logerr(
                        "chunk %s: upload ok but archive move failed; will retry", chunk_id
                    )
                    return True
            except OSError as exc:
                rospy.logerr(
                    "chunk %s: upload ok but archive move failed: %s; will retry",
                    chunk_id,
                    exc,
                )
                return True
        else:
            remove_pose_chunk(self.pose_spool_dir, self.robot_id, chunk_id)
        return True

    def _retry_pending_archives(self) -> None:
        if not self.retain_after_upload:
            return
        for chunk_id in find_pending_archive_pose_chunks(self.pose_spool_dir, self.robot_id):
            try:
                if archive_pose_chunk(
                    self.archive_dir, self.pose_spool_dir, self.robot_id, chunk_id
                ):
                    rospy.loginfo("archived pending pose chunk=%s", chunk_id)
                else:
                    rospy.logwarn_throttle(
                        60.0, "archive retry pending for pose chunk=%s", chunk_id
                    )
            except OSError as exc:
                rospy.logwarn_throttle(
                    60.0, "archive retry failed pose chunk=%s: %s", chunk_id, exc
                )

    def _upload_chunk(self, chunk_id: str) -> bool:
        if not self._upload_enabled or rospy.is_shutdown():
            return False

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

        self._finalize_after_upload(chunk_id, resp.status_code)
        rospy.loginfo(
            "uploaded pose chunk=%s (%d rows)",
            chunk_id,
            meta.get("row_count", 0),
        )
        return True

    def spin(self):
        while not rospy.is_shutdown() and self._upload_enabled:
            self._retry_pending_archives()

            if not self._central_reachable():
                rospy.logwarn_throttle(60.0, "central ingest unreachable; will retry")
                if rospy.sleep(self._backoff_s):
                    break
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)
                continue

            self._backoff_s = self.poll_interval_s
            chunks = find_ready_pose_chunks(self.pose_spool_dir, self.robot_id)
            if not chunks:
                if rospy.sleep(self.poll_interval_s):
                    break
                continue

            for chunk_id in chunks:
                if rospy.is_shutdown() or not self._upload_enabled:
                    break
                if self._upload_chunk(chunk_id):
                    continue
                if rospy.sleep(min(self._backoff_s, self.poll_interval_s)):
                    break
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)

            if rospy.is_shutdown() or not self._upload_enabled:
                break
            if rospy.sleep(self.poll_interval_s):
                break


if __name__ == "__main__":
    PoseUploader().spin()
