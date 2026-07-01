#!/usr/bin/env python3
"""Record detected objects (map-frame poses) to local SQLite chunks for later upload."""

from __future__ import annotations

import json
import os
import sys
import threading

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from capture_utils.detections import detected_objects_to_list
from capture_utils.manifest import utc_now_iso
from capture_utils.pose_lookup import lookup_pose
from capture_utils.detection_spool import DetectionChunkWriter, detection_spool_root
from capture_utils.spool import dir_size_bytes
from dds_utils import RobotIdError, require_robot_id_int
from mattbot_image_detection.msg import DetectedObjectArray


class DetectionLogger:
    def __init__(self):
        rospy.init_node("detection_logger", anonymous=False)

        try:
            self.robot_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        self.detection_spool_dir = rospy.get_param(
            "~detection_spool_dir", "/workspace/catkin_ws/data/detection_spool"
        )
        self.max_detection_spool_bytes = int(
            rospy.get_param("~max_detection_spool_bytes", 5 * 1024 ** 3)
        )
        self.chunk_hours = float(rospy.get_param("~chunk_hours", 1.0))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.detections_topic = rospy.get_param("~detections_topic", "/detected_objects")
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.0))
        self.sample_hz = float(rospy.get_param("~sample_hz", 0.0))

        os.makedirs(detection_spool_root(self.detection_spool_dir, self.robot_id), exist_ok=True)

        self._lock = threading.Lock()
        self._latest_pose = None
        self._last_sample_wall = 0.0
        self._tf_listener = tf.TransformListener()
        self._writer = DetectionChunkWriter(self.detection_spool_dir, self.robot_id)

        rospy.Subscriber(
            self.detections_topic, DetectedObjectArray, self._detections_callback, queue_size=10
        )
        if self.pose_topic:
            rospy.Subscriber(
                self.pose_topic, PoseWithCovarianceStamped, self._pose_callback, queue_size=1
            )

        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "detection_logger ready robot_id=%s spool=%s topic=%s min_conf=%.2f sample_hz=%.2f",
            self.robot_id,
            self.detection_spool_dir,
            self.detections_topic,
            self.min_confidence,
            self.sample_hz,
        )

    def _should_record(self, now: float) -> bool:
        if self.sample_hz <= 0.0:
            return True
        if self._last_sample_wall <= 0.0:
            return True
        return (now - self._last_sample_wall) >= (1.0 / self.sample_hz)

    def _detections_callback(self, msg: DetectedObjectArray):
        objects = detected_objects_to_list(msg, self.min_confidence)
        if not objects:
            return

        if self._spool_over_limit():
            rospy.logwarn_throttle(60.0, "detection spool size limit exceeded; dropping samples")
            return

        now = rospy.get_time()
        if not self._should_record(now):
            return

        with self._lock:
            latest_pose = self._latest_pose
        robot_pose = lookup_pose(self._tf_listener, self.map_frame, self.base_frame, latest_pose)

        try:
            self._writer.insert(self._build_row(msg, robot_pose, objects))
        except Exception as exc:
            rospy.logwarn_throttle(30.0, "detection insert failed: %s", exc)
            return

        self._last_sample_wall = now

        if self._writer.maybe_rotate(self.chunk_hours):
            self._writer.reopen()

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._latest_pose = msg

    def _spool_over_limit(self) -> bool:
        root = detection_spool_root(self.detection_spool_dir, self.robot_id)
        return os.path.isdir(root) and dir_size_bytes(root) >= self.max_detection_spool_bytes

    def _build_row(
        self,
        detections_msg: DetectedObjectArray,
        robot_pose: dict | None,
        objects: list[dict],
    ) -> dict:
        row = {
            "wall_time": utc_now_iso(),
            "ros_time_sec": detections_msg.header.stamp.secs,
            "ros_time_nsec": detections_msg.header.stamp.nsecs,
            "robot_valid": 0,
            "robot_x": None,
            "robot_y": None,
            "robot_theta": None,
            "robot_frame": None,
            "object_count": len(objects),
            "objects_json": json.dumps(objects),
        }
        if robot_pose is not None:
            row["robot_valid"] = 1
            row["robot_x"] = robot_pose["x"]
            row["robot_y"] = robot_pose["y"]
            row["robot_theta"] = robot_pose["theta"]
            row["robot_frame"] = robot_pose["frame"]
        return row

    def _shutdown(self):
        if self._writer is not None:
            row_count = self._writer.close()
            self._writer = None
            if row_count > 0:
                rospy.loginfo(
                    "detection chunk sealed locally (%d rows); upload deferred", row_count
                )


if __name__ == "__main__":
    DetectionLogger()
    rospy.spin()
