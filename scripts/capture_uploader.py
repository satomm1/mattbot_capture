#!/usr/bin/env python3
"""Upload ready capture sessions from local spool to central ingest service."""

from __future__ import annotations

import os
import sys
import time

import rospy
import requests

from capture_utils import (
    find_ready_sessions,
    ingest_health_url,
    ingest_upload_url,
    load_manifest,
    manifest_path,
    remove_session,
    session_dir,
)
from dds_utils import RobotIdError, require_robot_id_int


def _mime_type(filename: str) -> str:
    if filename.lower().endswith(".wav"):
        return "audio/wav"
    return "image/jpeg"


class CaptureUploader:
    def __init__(self):
        rospy.init_node("capture_uploader", anonymous=False)

        try:
            self.robot_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        self.spool_dir = rospy.get_param("~spool_dir", "/workspace/catkin_ws/data/capture_spool")
        ingest_ip = rospy.get_param("~ingest_ip", "192.168.50.2")
        ingest_port = int(rospy.get_param("~ingest_port", 8080))
        ingest_scheme = rospy.get_param("~ingest_scheme", "http")
        self.ingest_url = ingest_upload_url(ingest_ip, ingest_port, ingest_scheme)
        self.health_url = ingest_health_url(ingest_ip, ingest_port, ingest_scheme)
        self.api_key = rospy.get_param("~api_key", "")
        self.poll_interval_s = float(rospy.get_param("~poll_interval_s", 30.0))
        self.request_timeout_s = float(rospy.get_param("~request_timeout_s", 120.0))
        self.retry_max_s = float(rospy.get_param("~retry_max_s", 300.0))
        self.check_health = bool(rospy.get_param("~check_health", True))

        self._backoff_s = self.poll_interval_s

        rospy.loginfo(
            "capture_uploader ready robot_id=%s ingest=%s spool=%s",
            self.robot_id,
            self.ingest_url,
            self.spool_dir,
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

    def _upload_session(self, session_id: str) -> bool:
        manifest_file = manifest_path(self.spool_dir, self.robot_id, session_id)
        try:
            manifest = load_manifest(manifest_file)
        except (OSError, ValueError) as exc:
            rospy.logerr("skip session %s: bad manifest: %s", session_id, exc)
            return False

        if manifest.get("status") != "ready_for_upload":
            return False

        session_path = session_dir(self.spool_dir, self.robot_id, session_id)
        opened = []
        resp = None
        try:
            files = []
            for frame in manifest.get("frames", []):
                filename = frame.get("filename")
                if not filename:
                    rospy.logerr("session %s: frame missing filename", session_id)
                    return False
                image_path = os.path.join(session_path, filename)
                if not os.path.isfile(image_path):
                    rospy.logerr("session %s: missing file %s", session_id, filename)
                    return False
                handle = open(image_path, "rb")
                opened.append(handle)
                files.append(("files", (filename, handle, _mime_type(filename))))

            with open(manifest_file, "rb") as manifest_handle:
                multipart = [("manifest", ("manifest.json", manifest_handle, "application/json"))]
                multipart.extend(files)
                resp = requests.post(
                    self.ingest_url,
                    headers=self._headers(),
                    files=multipart,
                    timeout=self.request_timeout_s,
                )
        except requests.RequestException as exc:
            rospy.logwarn("upload failed session=%s: %s", session_id, exc)
            return False
        finally:
            for handle in opened:
                handle.close()

        if resp is None or resp.status_code not in (200, 201):
            status = resp.status_code if resp is not None else "none"
            body = resp.text[:500] if resp is not None else ""
            rospy.logwarn(
                "upload rejected session=%s status=%s body=%s",
                session_id,
                status,
                body,
            )
            return False

        remove_session(self.spool_dir, self.robot_id, session_id)
        rospy.loginfo("uploaded session=%s (%d frames)", session_id, len(manifest.get("frames", [])))
        return True

    def spin(self):
        while not rospy.is_shutdown():
            if not self._central_reachable():
                rospy.logwarn_throttle(60.0, "central ingest unreachable; will retry")
                time.sleep(self._backoff_s)
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)
                continue

            self._backoff_s = self.poll_interval_s
            sessions = find_ready_sessions(self.spool_dir, self.robot_id)
            if not sessions:
                time.sleep(self.poll_interval_s)
                continue

            for session_id in sessions:
                if rospy.is_shutdown():
                    break
                if self._upload_session(session_id):
                    continue
                time.sleep(min(self._backoff_s, self.poll_interval_s))
                self._backoff_s = min(self.retry_max_s, self._backoff_s * 2.0)

            time.sleep(self.poll_interval_s)


if __name__ == "__main__":
    CaptureUploader().spin()
