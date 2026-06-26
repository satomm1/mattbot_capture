"""Shared TF / amcl_pose lookup for capture and pose logger."""

from __future__ import annotations

import rospy
import tf
from tf.transformations import euler_from_quaternion


def lookup_pose(tf_listener, map_frame, base_frame, latest_pose_msg) -> dict | None:
    """Return {x, y, theta, frame} from TF, or amcl fallback, or None."""
    try:
        pos, rot = tf_listener.lookupTransform(map_frame, base_frame, rospy.Time(0))
        _roll, _pitch, yaw = euler_from_quaternion(rot)
        return {"x": pos[0], "y": pos[1], "theta": yaw, "frame": map_frame}
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

    if latest_pose_msg is None:
        return None

    p = latest_pose_msg.pose.pose.position
    q = latest_pose_msg.pose.pose.orientation
    _roll, _pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    frame = latest_pose_msg.header.frame_id or map_frame
    return {"x": p.x, "y": p.y, "theta": yaw, "frame": frame}
