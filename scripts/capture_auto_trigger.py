#!/usr/bin/env python3
"""Start/stop capture sessions automatically when navigation conditions are met.

Watches /robot_mode from localize_and_navigate.py and calls the existing
/capture/start_session and /capture/stop_session services. Requires capture_node
to be running (launch capture.launch with enabled:=true).
"""

from __future__ import annotations

import rospy
from std_msgs.msg import Int32

from mattbot_capture.srv import CaptureStartSession, CaptureStopSession

# localize_and_navigate.py Mode enum (values in ~nav_capture_modes by default):
#   3=ALIGN  4=TRACK  5=PARK_POSE  6=PARK_HEADING  7=BACKING
#   10=STOPPED_FOR_PERSON  11=STOPPED_FOR_AGENT
# Excluded: 0=IDLE, 1/2=LOCALIZING*, 8=WAITING_FOR_INIT, 9=RELOCALIZING,
#           12=MULTIAGENT_CONTROL_COMPUTING (robot held still for fleet timing)


class CaptureAutoTrigger:
    def __init__(self):
        rospy.init_node("capture_auto_trigger", anonymous=False)

        self.auto_capture_enabled = bool(rospy.get_param("~auto_capture_enabled", True))
        self.nav_sample_hz = float(rospy.get_param("~nav_sample_hz", 0.5))
        robot_mode_topic = rospy.get_param("~robot_mode_topic", "/robot_mode")
        nav_modes = rospy.get_param("~nav_capture_modes", [3, 4, 5, 6, 7, 10, 11])
        self._nav_capture_modes = set(int(m) for m in nav_modes)

        # True only when this node started the current capture session.
        # Prevents stop_session from finalizing a manually started session.
        self._auto_session_active = False
        self._nav_active = False
        self._manual_session_warned = False

        rospy.wait_for_service("/capture/start_session")
        rospy.wait_for_service("/capture/stop_session")
        self._start_session = rospy.ServiceProxy("/capture/start_session", CaptureStartSession)
        self._stop_session = rospy.ServiceProxy("/capture/stop_session", CaptureStopSession)

        rospy.Subscriber(robot_mode_topic, Int32, self._robot_mode_callback, queue_size=10)

        rospy.loginfo(
            "capture_auto_trigger ready auto=%s hz=%s modes=%s",
            self.auto_capture_enabled,
            self.nav_sample_hz,
            sorted(self._nav_capture_modes),
        )

    def _robot_mode_callback(self, msg: Int32):
        self._nav_active = msg.data in self._nav_capture_modes
        self._sync_capture_state()

    def _sync_capture_state(self):
        if not self.auto_capture_enabled:
            return

        # Future conditions: add subscriber + bool flag, OR into should_capture below
        should_capture = self._nav_active

        if should_capture and not self._auto_session_active:
            self._start_auto_session()
        elif not should_capture and self._auto_session_active:
            self._stop_auto_session()

    def _start_auto_session(self):
        if self.nav_sample_hz <= 0.0:
            rospy.logwarn("nav_sample_hz <= 0; auto-capture disabled")
            return

        try:
            resp = self._start_session("navigation", "", self.nav_sample_hz)
        except rospy.ServiceException as exc:
            rospy.logwarn("start_session failed: %s", exc)
            return

        if resp.success:
            self._auto_session_active = True
            self._manual_session_warned = False
            rospy.loginfo("auto-capture started session=%s", resp.session_id)
            return

        # capture_node rejects start when a manual session is already active
        if not self._manual_session_warned:
            rospy.logwarn("auto-capture skipped: %s", resp.message)
            self._manual_session_warned = True

    def _stop_auto_session(self):
        try:
            resp = self._stop_session()
        except rospy.ServiceException as exc:
            rospy.logwarn("stop_session failed: %s", exc)
            return

        self._auto_session_active = False
        if resp.success:
            rospy.loginfo("auto-capture stopped: %s", resp.message)
        else:
            rospy.logwarn("auto-capture stop failed: %s", resp.message)


if __name__ == "__main__":
    CaptureAutoTrigger()
    rospy.spin()
