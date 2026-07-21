#!/usr/bin/env python3
"""Capture camera frames to local spool via ROS services and topics."""

from __future__ import annotations

import json
import os
import sys
import threading
import time

import cv2
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from capture_utils.pose_lookup import lookup_pose
from capture_utils.camera_orient import orient_bgr, orient_depth_u16, orient_mono8

from capture_utils.detections import detected_objects_to_list
from capture_utils import (
    SessionWriter,
    SpoolError,
    dir_size_bytes,
    depth_frame_filename,
    frame_filename,
    ir_frame_filename,
    new_session_id,
    robot_spool_root,
)
from dds_utils import RobotIdError, require_robot_id_int
from mattbot_capture.srv import (
    CaptureSaveFrame,
    CaptureSaveFrameResponse,
    CaptureStartSession,
    CaptureStartSessionResponse,
    CaptureStopSession,
    CaptureStopSessionResponse,
)
from mattbot_image_detection.msg import DetectedObjectArray


class CaptureNode:
    def __init__(self):
        rospy.init_node("capture_node", anonymous=False)

        try:
            self.robot_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        self.spool_dir = rospy.get_param("~spool_dir", "/workspace/catkin_ws/data/capture_spool")
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.max_spool_bytes = int(rospy.get_param("~max_spool_bytes", 5 * 1024 ** 3))
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.detections_topic = rospy.get_param("~detections_topic", "/detected_objects")
        self.capture_ir = bool(rospy.get_param("~capture_ir", True))
        self.ir_image_topic = rospy.get_param("~ir_image_topic", "/camera/ir/image_raw")
        self.capture_depth = bool(rospy.get_param("~capture_depth", True))
        self.depth_image_topic = rospy.get_param("~depth_image_topic", "/camera/depth/image_raw")
        self.tall = bool(rospy.get_param("~tall", False))
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        # 0 = no rotation; continuous capture sets e.g. 20 to seal bounded manifests
        self.session_chunk_seconds = float(rospy.get_param("~session_chunk_seconds", 0.0))
        self.log_save_timing = bool(rospy.get_param("~log_save_timing", False))

        os.makedirs(robot_spool_root(self.spool_dir, self.robot_id), exist_ok=True)

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._latest_image = None  # sensor_msgs/Image
        self._latest_ir_image = None  # sensor_msgs/Image (mono8)
        self._latest_depth_image = None  # sensor_msgs/Image (16UC1)
        self._latest_detections = None  # DetectedObjectArray
        self._latest_pose = None  # PoseWithCovarianceStamped
        self._tf_listener = tf.TransformListener()

        self._session = None  # SessionWriter | None
        self._session_trigger = ""
        self._session_sample_hz = 0.0
        self._chunk_started_wall = 0.0
        self._sample_timer = None
        self._last_saved_stamp = None  # (sec, nsec) used in last filename
        self._ir_warned = False
        self._depth_warned = False

        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        # IR optional: camera enable_ir mirrors capture at bringup; RGB-only if disabled
        if self.capture_ir and self.ir_image_topic:
            rospy.Subscriber(self.ir_image_topic, Image, self._ir_image_callback, queue_size=1)
            rospy.Timer(rospy.Duration(5.0), self._ir_startup_check, oneshot=True)
        # Depth always on for OSOD; capture_node saves companion PNG when capture_depth
        if self.capture_depth and self.depth_image_topic:
            rospy.Subscriber(self.depth_image_topic, Image, self._depth_image_callback, queue_size=1)
            rospy.Timer(rospy.Duration(5.0), self._depth_startup_check, oneshot=True)
        if self.pose_topic:
            rospy.Subscriber(
                self.pose_topic, PoseWithCovarianceStamped, self._pose_callback, queue_size=1
            )
        if self.detections_topic:
            rospy.Subscriber(
                self.detections_topic,
                DetectedObjectArray,
                self._detections_callback,
                queue_size=1,
            )

        rospy.Subscriber("/capture/save_frame", Empty, self._save_frame_topic_callback, queue_size=10)

        rospy.Service("/capture/start_session", CaptureStartSession, self._start_session_handler)
        rospy.Service("/capture/stop_session", CaptureStopSession, self._stop_session_handler)
        rospy.Service("/capture/save_frame", CaptureSaveFrame, self._save_frame_handler)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "capture_node ready robot_id=%s spool=%s tall=%s chunk_s=%.1f",
            self.robot_id,
            self.spool_dir,
            self.tall,
            self.session_chunk_seconds,
        )

    def _image_callback(self, msg: Image):
        with self._lock:
            self._latest_image = msg

    def _ir_image_callback(self, msg: Image):
        with self._lock:
            self._latest_ir_image = msg

    def _depth_image_callback(self, msg: Image):
        with self._lock:
            self._latest_depth_image = msg

    def _ir_startup_check(self, _event):
        if self._ir_warned:
            return
        with self._lock:
            has_ir = self._latest_ir_image is not None
        if not has_ir:
            self._ir_warned = True
            rospy.logwarn(
                "capture_ir enabled but no IR frames on %s yet (enable_ir:=capture at bringup?)",
                self.ir_image_topic,
            )

    def _depth_startup_check(self, _event):
        if self._depth_warned:
            return
        with self._lock:
            has_depth = self._latest_depth_image is not None
        if not has_depth:
            self._depth_warned = True
            rospy.logwarn(
                "capture_depth enabled but no depth frames on %s yet",
                self.depth_image_topic,
            )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._latest_pose = msg

    def _detections_callback(self, msg: DetectedObjectArray):
        with self._lock:
            self._latest_detections = msg

    def _save_frame_topic_callback(self, _msg: Empty):
        self._save_frame(metadata_json="")

    def _start_session_handler(self, req: CaptureStartSession):
        resp = CaptureStartSessionResponse(success=False, session_id="", message="")
        with self._lock:
            if self._session is not None:
                resp.message = "session already active; stop it first"
                return resp

            session_id = req.session_id.strip() or new_session_id()
            trigger = req.trigger.strip() or "manual"
            try:
                self._session = SessionWriter(self.spool_dir, self.robot_id, session_id, trigger)
            except OSError as exc:
                resp.message = f"failed to create session: {exc}"
                return resp

            self._session_trigger = trigger
            self._session_sample_hz = max(0.0, float(req.sample_hz))
            self._chunk_started_wall = rospy.get_time()
            self._configure_sample_timer()

        resp.success = True
        resp.session_id = session_id
        resp.message = "session started"
        rospy.loginfo(
            "capture session started id=%s trigger=%s hz=%s chunk_s=%.1f",
            session_id,
            trigger,
            self._session_sample_hz,
            self.session_chunk_seconds,
        )
        return resp

    def _stop_session_handler(self, _req: CaptureStopSession):
        resp = CaptureStopSessionResponse(success=False, message="")
        with self._lock:
            if self._session is None:
                resp.message = "no active session"
                return resp

            session_id = self._session.session_id
            try:
                self._session.finalize()
            except SpoolError as exc:
                resp.message = str(exc)
                return resp
            finally:
                self._clear_session()

        resp.success = True
        resp.message = f"session {session_id} finalized"
        rospy.loginfo("capture session stopped id=%s", session_id)
        return resp

    def _save_frame_handler(self, req: CaptureSaveFrame):
        frame_id, err = self._save_frame(req.metadata_json)
        resp = CaptureSaveFrameResponse(success=frame_id is not None, frame_id=frame_id or "", message=err or "ok")
        return resp

    def _configure_sample_timer(self):
        if self._sample_timer is not None:
            self._sample_timer.shutdown()
            self._sample_timer = None
        if self._session is not None and self._session_sample_hz > 0.0:
            period = 1.0 / self._session_sample_hz
            self._sample_timer = rospy.Timer(rospy.Duration(period), self._sample_timer_callback)

    def _sample_timer_callback(self, _event):
        self._maybe_rotate_chunk()
        self._save_frame(metadata_json="")

    def _maybe_rotate_chunk(self):
        """Finalize current session and open a new one when chunk duration elapses."""
        if self.session_chunk_seconds <= 0.0:
            return

        with self._lock:
            if self._session is None:
                return
            now = rospy.get_time()
            if self._chunk_started_wall <= 0.0:
                self._chunk_started_wall = now
                return
            if (now - self._chunk_started_wall) < self.session_chunk_seconds:
                return

            old_id = self._session.session_id
            trigger = self._session_trigger or "manual"
            try:
                self._session.finalize()
            except SpoolError as exc:
                rospy.logwarn("chunk finalize failed session=%s: %s", old_id, exc)
                return

            session_id = new_session_id()
            try:
                self._session = SessionWriter(self.spool_dir, self.robot_id, session_id, trigger)
            except OSError as exc:
                rospy.logerr("chunk rotate failed to open session: %s", exc)
                self._clear_session()
                return

            self._session_trigger = trigger
            self._chunk_started_wall = now
            rospy.loginfo(
                "capture chunk rotated sealed=%s new=%s trigger=%s",
                old_id,
                session_id,
                trigger,
            )

    def _clear_session(self):
        self._session = None
        self._session_trigger = ""
        self._session_sample_hz = 0.0
        self._chunk_started_wall = 0.0
        self._last_saved_stamp = None
        if self._sample_timer is not None:
            self._sample_timer.shutdown()
            self._sample_timer = None

    def _unique_frame_stamp(self, msg: Image) -> tuple[int, int]:
        """Return (sec, nsec) for filenames; fall back to now if camera stamp stalls.

        Continuous sampling can fire faster than the camera stamp advances (or with a
        stuck header stamp). Reusing that stamp overwrites the same JPEG on disk while
        manifest wall_time keeps updating — so uniquify when the stamp does not move.
        """
        cam = msg.header.stamp
        cam_key = (int(cam.secs), int(cam.nsecs))
        if cam.to_sec() > 0.0 and cam_key != self._last_saved_stamp:
            stamp_key = cam_key
        else:
            now = rospy.Time.now()
            stamp_key = (int(now.secs), int(now.nsecs))
            if stamp_key == self._last_saved_stamp:
                stamp_key = (stamp_key[0], stamp_key[1] + 1)
            rospy.logwarn_throttle(
                30.0,
                "image stamp not advancing (last=%s); naming frames with rospy.Time.now()",
                self._last_saved_stamp,
            )
        self._last_saved_stamp = stamp_key
        return stamp_key

    def _shutdown(self):
        with self._lock:
            if self._sample_timer is not None:
                self._sample_timer.shutdown()
                self._sample_timer = None
            if self._session is None:
                return
            session_id = self._session.session_id
            try:
                self._session.finalize()
                rospy.loginfo("capture session finalized on shutdown id=%s", session_id)
            except SpoolError as exc:
                rospy.logwarn("shutdown finalize failed session=%s: %s", session_id, exc)
            self._session = None
            self._session_trigger = ""
            self._session_sample_hz = 0.0
            self._chunk_started_wall = 0.0

    def _spool_over_limit(self) -> bool:
        root = robot_spool_root(self.spool_dir, self.robot_id)
        if not os.path.isdir(root):
            return False
        return dir_size_bytes(root) >= self.max_spool_bytes

    def _encode_jpeg(self, msg: Image):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            return None, f"cv_bridge error: {exc}"
        cv_image = orient_bgr(cv_image, self.tall)
        ok, encoded = cv2.imencode(
            ".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return None, "jpeg encode failed"
        return encoded.tobytes(), None

    def _encode_ir_jpeg(self, msg: Image):
        # mono8 grayscale -> JPEG (same quality as RGB)
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError as exc:
            return None, f"cv_bridge error: {exc}"
        cv_image = orient_mono8(cv_image, self.tall)
        ok, encoded = cv2.imencode(
            ".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return None, "ir jpeg encode failed"
        return encoded.tobytes(), None

    def _encode_depth_png(self, msg: Image):
        # 16UC1 millimeters -> lossless PNG (JPEG would destroy metric depth)
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        except CvBridgeError as exc:
            return None, f"cv_bridge error: {exc}"
        cv_image = orient_depth_u16(cv_image, self.tall)
        ok, encoded = cv2.imencode(".png", cv_image)
        if not ok:
            return None, "depth png encode failed"
        return encoded.tobytes(), None

    def _lookup_pose(self) -> dict | None:
        with self._lock:
            pose_msg = self._latest_pose
        return lookup_pose(self._tf_listener, self.map_frame, self.base_frame, pose_msg)

    def _detections_to_list(self) -> list:
        with self._lock:
            msg = self._latest_detections
        return detected_objects_to_list(msg)

    def _parse_extra(self, metadata_json: str) -> dict:
        if not metadata_json or not metadata_json.strip():
            return {}
        try:
            parsed = json.loads(metadata_json)
        except json.JSONDecodeError as exc:
            rospy.logwarn("invalid metadata_json: %s", exc)
            return {"metadata_parse_error": str(exc)}
        if isinstance(parsed, dict):
            return parsed
        return {"metadata": parsed}

    def _save_frame(self, metadata_json: str):
        if self._spool_over_limit():
            return None, "spool size limit exceeded"

        with self._lock:
            msg = self._latest_image
        if msg is None:
            return None, "no camera frame available yet"

        t0 = time.perf_counter() if self.log_save_timing else None

        jpeg_bytes, err = self._encode_jpeg(msg)
        if err:
            return None, err

        ros_sec, ros_nsec = self._unique_frame_stamp(msg)
        filename = frame_filename(ros_sec, ros_nsec)
        pose = self._lookup_pose()
        detections = self._detections_to_list()
        extra = self._parse_extra(metadata_json)
        cam = msg.header.stamp
        if (int(cam.secs), int(cam.nsecs)) != (ros_sec, ros_nsec) and cam.to_sec() > 0.0:
            extra.setdefault("camera_stamp", {"sec": int(cam.secs), "nsec": int(cam.nsecs)})

        with self._lock:
            one_shot = self._session is None
            if one_shot:
                session_id = new_session_id()
                trigger = extra.pop("trigger", "snapshot")
                if not isinstance(trigger, str) or not trigger.strip():
                    trigger = "snapshot"
                try:
                    session = SessionWriter(self.spool_dir, self.robot_id, session_id, trigger)
                except OSError as exc:
                    return None, f"failed to create session: {exc}"
            else:
                session = self._session

        try:
            frame_id = session.append_frame(
                jpeg_bytes, filename, ros_sec, ros_nsec, pose, detections, extra
            )
        except SpoolError as exc:
            return None, str(exc)

        # Paired IR: latest cached frame at save tick, linked to RGB via rgb_frame_id
        if self.capture_ir:
            with self._lock:
                ir_msg = self._latest_ir_image
            if ir_msg is not None:
                ir_jpeg, ir_err = self._encode_ir_jpeg(ir_msg)
                if ir_err:
                    rospy.logwarn("IR companion skipped: %s", ir_err)
                else:
                    ir_filename = ir_frame_filename(ros_sec, ros_nsec)
                    ir_extra = {
                        "modality": "ir",
                        "content_type": "image/jpeg",
                        "rgb_frame_id": frame_id,
                    }
                    try:
                        session.append_frame(
                            ir_jpeg, ir_filename, ros_sec, ros_nsec, pose, [], ir_extra
                        )
                    except SpoolError as exc:
                        rospy.logwarn("IR companion save failed: %s", exc)

        # Paired depth: latest cached frame at save tick, linked to RGB via rgb_frame_id
        if self.capture_depth:
            with self._lock:
                depth_msg = self._latest_depth_image
            if depth_msg is not None:
                depth_png, depth_err = self._encode_depth_png(depth_msg)
                if depth_err:
                    rospy.logwarn("Depth companion skipped: %s", depth_err)
                else:
                    depth_filename = depth_frame_filename(ros_sec, ros_nsec)
                    depth_extra = {
                        "modality": "depth",
                        "content_type": "image/png",
                        "depth_encoding": "16UC1",
                        "depth_units": "millimeters",
                        "rgb_frame_id": frame_id,
                    }
                    try:
                        session.append_frame(
                            depth_png, depth_filename, ros_sec, ros_nsec, pose, [], depth_extra
                        )
                    except SpoolError as exc:
                        rospy.logwarn("Depth companion save failed: %s", exc)

        if one_shot:
            session.finalize()
            rospy.loginfo("one-shot capture saved session=%s frame=%s", session.session_id, frame_id)

        if t0 is not None:
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            rospy.loginfo(
                "capture save timing frame=%s elapsed_ms=%.1f (encode+write rgb/ir/depth)",
                frame_id,
                elapsed_ms,
            )

        return frame_id, None


if __name__ == "__main__":
    node = CaptureNode()
    rospy.spin()
