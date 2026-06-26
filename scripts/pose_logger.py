#!/usr/bin/env python3
"""Record robot pose locally to SQLite chunks for later upload."""

from __future__ import annotations

import math
import os
import sys
import threading

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Int32

from capture_utils.manifest import utc_now_iso
from capture_utils.pose_lookup import lookup_pose
from capture_utils.pose_spool import PoseChunkWriter, pose_spool_root
from capture_utils.spool import dir_size_bytes
from dds_utils import ROS_TOPIC_TRANSFORMATION_MATRIX, RobotIdError, TransformMixin, require_robot_id_int


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return abs(d)


class PoseLogger(TransformMixin):
    def __init__(self):
        rospy.init_node("pose_logger", anonymous=False)

        try:
            self.robot_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        self.init_transform_state()

        self.pose_spool_dir = rospy.get_param("~pose_spool_dir", "/workspace/catkin_ws/data/pose_spool")
        self.max_pose_spool_bytes = int(rospy.get_param("~max_pose_spool_bytes", 5 * 1024 ** 3))
        self.moving_sample_hz = float(rospy.get_param("~moving_sample_hz", 2.0))
        self.static_sample_hz = float(rospy.get_param("~static_sample_hz", 1.0 / 60.0))
        self.max_gap_s = float(rospy.get_param("~max_gap_s", 30.0))
        self.min_distance_m = float(rospy.get_param("~min_distance_m", 0.1))
        self.min_angle_rad = float(rospy.get_param("~min_angle_rad", 0.09))
        self.chunk_hours = float(rospy.get_param("~chunk_hours", 1.0))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")
        self.robot_mode_topic = rospy.get_param("~robot_mode_topic", "/robot_mode")

        os.makedirs(pose_spool_root(self.pose_spool_dir, self.robot_id), exist_ok=True)

        self._lock = threading.Lock()
        self._latest_pose = None
        self._robot_mode = 0
        self._tf_listener = tf.TransformListener()
        self._writer = PoseChunkWriter(self.pose_spool_dir, self.robot_id)

        self._last_sample_wall = 0.0
        self._last_saved_pose = None  # (x, y, theta) when valid

        rospy.Subscriber(
            ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback
        )
        if self.pose_topic:
            rospy.Subscriber(
                self.pose_topic, PoseWithCovarianceStamped, self._pose_callback, queue_size=1
            )
        rospy.Subscriber(self.robot_mode_topic, Int32, self._robot_mode_callback, queue_size=1)

        period = 1.0 / max(self.moving_sample_hz, 0.1)
        self._timer = rospy.Timer(rospy.Duration(period), self._tick)
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "pose_logger ready robot_id=%s spool=%s hz=%.2f",
            self.robot_id,
            self.pose_spool_dir,
            self.moving_sample_hz,
        )

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            self._latest_pose = msg

    def _robot_mode_callback(self, msg: Int32):
        self._robot_mode = msg.data

    def _spool_over_limit(self) -> bool:
        root = pose_spool_root(self.pose_spool_dir, self.robot_id)
        return os.path.isdir(root) and dir_size_bytes(root) >= self.max_pose_spool_bytes

    def _is_static(self) -> bool:
        return self._robot_mode == 0

    def _should_sample(self, now: float, pose: dict | None) -> bool:
        if self._last_sample_wall <= 0:
            return True

        gap = now - self._last_sample_wall
        if gap >= self.max_gap_s:
            return True

        is_static = self._is_static()
        min_interval = 1.0 / self.static_sample_hz if is_static else 1.0 / self.moving_sample_hz
        if gap >= min_interval:
            return True

        # Motion while throttled (e.g. static interval): save if moved significantly
        if pose is not None and self._last_saved_pose is not None:
            lx, ly, lt = self._last_saved_pose
            dist = math.hypot(pose["x"] - lx, pose["y"] - ly)
            if dist >= self.min_distance_m or _angle_diff(pose["theta"], lt) >= self.min_angle_rad:
                return True

        return False

    def _build_row(self, pose: dict | None) -> dict:
        now = rospy.Time.now()
        row = {
            "wall_time": utc_now_iso(),
            "ros_time_sec": now.secs,
            "ros_time_nsec": now.nsecs,
            "is_static": self._is_static(),
            "valid": 0,
            "x": None,
            "y": None,
            "theta": None,
            "frame": None,
            "ref_x": None,
            "ref_y": None,
            "ref_theta": None,
        }
        if pose is None:
            return row

        row["valid"] = 1
        row["x"] = pose["x"]
        row["y"] = pose["y"]
        row["theta"] = pose["theta"]
        row["frame"] = pose["frame"]
        if self.R is not None:
            ref = self.transform_point([pose["x"], pose["y"], pose["theta"]])
            row["ref_x"] = float(ref[0])
            row["ref_y"] = float(ref[1])
            row["ref_theta"] = float(ref[2])
        return row

    def _tick(self, _event):
        if self._spool_over_limit():
            rospy.logwarn_throttle(60.0, "pose spool size limit exceeded; dropping samples")
            return

        with self._lock:
            latest_pose = self._latest_pose
        pose = lookup_pose(self._tf_listener, self.map_frame, self.base_frame, latest_pose)

        now = rospy.get_time()
        if not self._should_sample(now, pose):
            return

        try:
            self._writer.insert(self._build_row(pose))
        except Exception as exc:
            rospy.logwarn_throttle(30.0, "pose insert failed: %s", exc)
            return

        self._last_sample_wall = now
        if pose is not None:
            self._last_saved_pose = (pose["x"], pose["y"], pose["theta"])

        if self._writer.maybe_rotate(self.chunk_hours):
            self._writer.reopen()

    def _shutdown(self):
        if self._timer is not None:
            self._timer.shutdown()
        if self._writer is not None:
            row_count = self._writer.close()
            self._writer = None
            if row_count > 0:
                rospy.loginfo("pose chunk sealed locally (%d rows); upload deferred", row_count)


if __name__ == "__main__":
    PoseLogger()
    rospy.spin()
