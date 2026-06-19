#!/usr/bin/env python3
"""Capture camera frames to local spool via ROS services and topics."""

from __future__ import annotations

import json
import os
import sys
import threading

import cv2
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tf.transformations import euler_from_quaternion

from capture_utils import (
    SessionWriter,
    SpoolError,
    dir_size_bytes,
    frame_filename,
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

        self.spool_dir = rospy.get_param("~spool_dir", "/var/robot_capture/spool")
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.max_spool_bytes = int(rospy.get_param("~max_spool_bytes", 5 * 1024 ** 3))
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.detections_topic = rospy.get_param("~detections_topic", "/detected_objects")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")

        os.makedirs(robot_spool_root(self.spool_dir, self.robot_id), exist_ok=True)

        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._latest_image = None  # sensor_msgs/Image
        self._latest_detections = None  # DetectedObjectArray
        self._latest_pose = None  # PoseWithCovarianceStamped
        self._tf_listener = tf.TransformListener()

        self._session = None  # SessionWriter | None
        self._session_sample_hz = 0.0
        self._sample_timer = None

        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
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

        rospy.loginfo(
            "capture_node ready robot_id=%s spool=%s", self.robot_id, self.spool_dir
        )

    def _image_callback(self, msg: Image):
        with self._lock:
            self._latest_image = msg

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

        self._session_sample_hz = max(0.0, float(req.sample_hz))
        self._configure_sample_timer()
        resp.success = True
        resp.session_id = session_id
        resp.message = "session started"
        rospy.loginfo("capture session started id=%s trigger=%s hz=%s", session_id, trigger, self._session_sample_hz)
        return resp

    def _stop_session_handler(self, _req: CaptureStopSession):
        resp = CaptureStopSessionResponse(success=False, message="")
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
        self._save_frame(metadata_json="")

    def _clear_session(self):
        self._session = None
        self._session_sample_hz = 0.0
        if self._sample_timer is not None:
            self._sample_timer.shutdown()
            self._sample_timer = None

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
        ok, encoded = cv2.imencode(
            ".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return None, "jpeg encode failed"
        return encoded.tobytes(), None

    def _lookup_pose(self) -> dict | None:
        try:
            (pos, rot) = self._tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0)
            )
            _roll, _pitch, yaw = euler_from_quaternion(rot)
            return {"x": pos[0], "y": pos[1], "theta": yaw, "frame": self.map_frame}
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        with self._lock:
            pose_msg = self._latest_pose
        if pose_msg is None:
            return None

        p = pose_msg.pose.pose.position
        q = pose_msg.pose.pose.orientation
        _roll, _pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        frame = pose_msg.header.frame_id or self.map_frame
        return {"x": p.x, "y": p.y, "theta": yaw, "frame": frame}

    def _detections_to_list(self) -> list:
        with self._lock:
            msg = self._latest_detections
        if msg is None:
            return []

        out = []
        for obj in msg.objects:
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

        jpeg_bytes, err = self._encode_jpeg(msg)
        if err:
            return None, err

        ros_sec = msg.header.stamp.secs
        ros_nsec = msg.header.stamp.nsecs
        filename = frame_filename(ros_sec, ros_nsec)
        pose = self._lookup_pose()
        detections = self._detections_to_list()
        extra = self._parse_extra(metadata_json)

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

        if one_shot:
            session.finalize()
            rospy.loginfo("one-shot capture saved session=%s frame=%s", session.session_id, frame_id)

        return frame_id, None


if __name__ == "__main__":
    node = CaptureNode()
    rospy.spin()
