#!/usr/bin/env python3
"""Start/stop capture sessions for navigation, person-detection, or continuous triggers.

Navigation: watches /robot_mode and records for the duration of each mission.
Person: watches /detected_objects, records while a person is visible, then for
person_tail_seconds after they leave. Nav takes priority when both could apply.
Continuous: after first /localized, records indefinitely (chunk rotation in capture_node).
When continuous is enabled, nav and person auto-capture are disabled.
Requires capture_node (capture.launch enabled:=true).
"""

from __future__ import annotations

import rospy
from std_msgs.msg import Int32

from capture_utils.localized_gate import LocalizedGate
from mattbot_capture.srv import CaptureStartSession, CaptureStopSession
from mattbot_image_detection.msg import DetectedObjectArray

# localize_and_navigate.py Mode enum (values in ~nav_capture_modes by default):
#   3=ALIGN  4=TRACK  5=PARK_POSE  6=PARK_HEADING  7=BACKING
#   10=STOPPED_FOR_PERSON  11=STOPPED_FOR_AGENT
# Excluded: 0=IDLE, 1/2=LOCALIZING*, 8=WAITING_FOR_INIT, 9=RELOCALIZING,
#           13=POSE_REFINE, 14=RECOVERY_VALIDATE,
#           12=MULTIAGENT_CONTROL_COMPUTING (robot held still for fleet timing)


class CaptureAutoTrigger:
    def __init__(self):
        rospy.init_node("capture_auto_trigger", anonymous=False)

        self.auto_capture_enabled = bool(rospy.get_param("~auto_capture_enabled", True))
        self.continuous_capture_enabled = bool(
            rospy.get_param("~continuous_capture_enabled", False)
        )
        self.continuous_sample_hz = float(rospy.get_param("~continuous_sample_hz", 10.0))
        self.require_localized = bool(rospy.get_param("~require_localized", True))
        localized_topic = rospy.get_param("~localized_topic", "/localized")

        self.nav_sample_hz = float(rospy.get_param("~nav_sample_hz", 0.5))
        self.person_capture_enabled = bool(rospy.get_param("~person_capture_enabled", True))
        self.person_sample_hz = float(rospy.get_param("~person_sample_hz", 1.0))
        self.person_tail_seconds = float(rospy.get_param("~person_tail_seconds", 5.0))
        self.person_min_confidence = float(rospy.get_param("~person_min_confidence", 0.5))
        robot_mode_topic = rospy.get_param("~robot_mode_topic", "/robot_mode")
        detections_topic = rospy.get_param("~detections_topic", "/detected_objects")
        nav_modes = rospy.get_param("~nav_capture_modes", [3, 4, 5, 6, 7, 10, 11])
        self._nav_capture_modes = set(int(m) for m in nav_modes)

        # Continuous mode owns the single capture_node session; disable nav/person.
        if self.continuous_capture_enabled:
            self.person_capture_enabled = False
            rospy.loginfo(
                "continuous capture enabled; nav/person auto-capture disabled"
            )

        # Only stop sessions this node started (not manual capture sessions).
        self._nav_session_active = False
        self._person_session_active = False
        self._continuous_session_active = False
        self._nav_active = False
        self._manual_session_warned = False

        # Person lifecycle: visible -> tail after last seen -> stop (re-detect cancels tail).
        self._person_last_seen = 0.0
        self._person_tail_timer = None
        self._continuous_poll_timer = None

        self._localized_gate = LocalizedGate(localized_topic, self.require_localized)

        rospy.wait_for_service("/capture/start_session")
        rospy.wait_for_service("/capture/stop_session")
        self._start_session = rospy.ServiceProxy("/capture/start_session", CaptureStartSession)
        self._stop_session = rospy.ServiceProxy("/capture/stop_session", CaptureStopSession)

        if self.continuous_capture_enabled:
            # Poll until localized, then start once; chunking is handled by capture_node.
            self._continuous_poll_timer = rospy.Timer(
                rospy.Duration(0.5), self._continuous_poll_callback
            )
        else:
            rospy.Subscriber(robot_mode_topic, Int32, self._robot_mode_callback, queue_size=10)
            rospy.Subscriber(
                detections_topic, DetectedObjectArray, self._detected_objects_callback, queue_size=10
            )

        if self.person_capture_enabled and self.person_sample_hz <= 0.0:
            rospy.logwarn("person_sample_hz <= 0; person auto-capture disabled")
            self.person_capture_enabled = False

        if self.continuous_capture_enabled and self.continuous_sample_hz <= 0.0:
            rospy.logwarn("continuous_sample_hz <= 0; continuous auto-capture disabled")
            self.continuous_capture_enabled = False

        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "capture_auto_trigger ready continuous=%s continuous_hz=%s nav_hz=%s "
            "person=%s person_hz=%s tail=%ss require_localized=%s",
            self.continuous_capture_enabled,
            self.continuous_sample_hz,
            self.nav_sample_hz,
            self.person_capture_enabled,
            self.person_sample_hz,
            self.person_tail_seconds,
            self.require_localized,
        )

    def _continuous_poll_callback(self, _event):
        if not self.auto_capture_enabled or not self.continuous_capture_enabled:
            return
        if self._continuous_session_active:
            return
        if not self._localized_gate.ready:
            return
        self._start_continuous_session()

    def _start_continuous_session(self):
        if self.continuous_sample_hz <= 0.0:
            return
        resp = self._try_start_session("continuous", self.continuous_sample_hz)
        if resp:
            self._continuous_session_active = True
            if self._continuous_poll_timer is not None:
                self._continuous_poll_timer.shutdown()
                self._continuous_poll_timer = None
            rospy.loginfo("continuous auto-capture started session=%s", resp.session_id)

    def _stop_continuous_session(self):
        if not self._continuous_session_active:
            return
        resp = self._try_stop_session()
        self._continuous_session_active = False
        if resp and resp.success:
            rospy.loginfo("continuous auto-capture stopped: %s", resp.message)

    def _robot_mode_callback(self, msg: Int32):
        self._nav_active = msg.data in self._nav_capture_modes
        self._sync_nav_capture()

    def _sync_nav_capture(self):
        if not self.auto_capture_enabled or self.continuous_capture_enabled:
            return

        if self._nav_active and not self._nav_session_active:
            # capture_node allows one session; stop person recording before nav starts.
            if self._person_session_active:
                self._stop_person_session()
            self._start_nav_session()
        elif not self._nav_active and self._nav_session_active:
            self._stop_nav_session()

    def _detected_objects_callback(self, msg: DetectedObjectArray):
        if (
            not self.auto_capture_enabled
            or self.continuous_capture_enabled
            or not self.person_capture_enabled
            or self._nav_active
        ):
            return

        # capture_node saves latest image/detections caches, not necessarily this message's frame.
        people = [
            obj
            for obj in msg.objects
            if obj.class_name == "person" and obj.probability >= self.person_min_confidence
        ]
        has_person = len(people) > 0

        if has_person:
            self._person_last_seen = rospy.get_time()
            self._cancel_person_tail_timer()
            if not self._person_session_active:
                self._start_person_session()
        elif self._person_session_active and self._person_tail_timer is None:
            self._schedule_person_tail()

    def _person_tail_callback(self, _event):
        self._person_tail_timer = None
        if self._person_session_active and rospy.get_time() - self._person_last_seen >= self.person_tail_seconds:
            self._stop_person_session()

    def _schedule_person_tail(self):
        self._cancel_person_tail_timer()
        self._person_tail_timer = rospy.Timer(
            rospy.Duration(self.person_tail_seconds),
            self._person_tail_callback,
            oneshot=True,
        )

    def _cancel_person_tail_timer(self):
        if self._person_tail_timer is not None:
            self._person_tail_timer.shutdown()
            self._person_tail_timer = None

    def _try_start_session(self, trigger: str, hz: float):
        try:
            resp = self._start_session(trigger, "", hz)
        except rospy.ServiceException as exc:
            rospy.logwarn("start_session failed: %s", exc)
            return None
        if resp.success:
            self._manual_session_warned = False
            return resp
        if not self._manual_session_warned:
            rospy.logwarn("auto-capture skipped: %s", resp.message)
            self._manual_session_warned = True
        return None

    def _try_stop_session(self):
        try:
            return self._stop_session()
        except rospy.ServiceException as exc:
            rospy.logwarn("stop_session failed: %s", exc)
            return None

    def _start_nav_session(self):
        if self.nav_sample_hz <= 0.0:
            return
        resp = self._try_start_session("navigation", self.nav_sample_hz)
        if resp:
            self._nav_session_active = True
            rospy.loginfo("nav auto-capture started session=%s", resp.session_id)

    def _stop_nav_session(self):
        resp = self._try_stop_session()
        self._nav_session_active = False
        if resp and resp.success:
            rospy.loginfo("nav auto-capture stopped: %s", resp.message)

    def _start_person_session(self):
        resp = self._try_start_session("person", self.person_sample_hz)
        if resp:
            self._person_session_active = True
            rospy.loginfo("person auto-capture started session=%s", resp.session_id)

    def _stop_person_session(self):
        self._cancel_person_tail_timer()
        if not self._person_session_active:
            return
        resp = self._try_stop_session()
        self._person_session_active = False
        if resp and resp.success:
            rospy.loginfo("person auto-capture stopped: %s", resp.message)

    def _shutdown(self):
        if self._continuous_poll_timer is not None:
            self._continuous_poll_timer.shutdown()
            self._continuous_poll_timer = None
        self._cancel_person_tail_timer()
        if self._continuous_session_active:
            self._stop_continuous_session()
        elif self._nav_session_active:
            self._stop_nav_session()
        elif self._person_session_active:
            self._stop_person_session()


if __name__ == "__main__":
    CaptureAutoTrigger()
    rospy.spin()
