"""Wait for first /localized before starting pose or detection spool writes."""

from __future__ import annotations

import rospy
from std_msgs.msg import Bool


class LocalizedGate:
    """Latch True on first localized message; optional bypass via require_localized=False."""

    def __init__(self, topic: str = "/localized", require: bool = True):
        self._require = require
        self._ready = not require
        if require:
            rospy.Subscriber(topic, Bool, self._callback, queue_size=1)

    def _callback(self, msg: Bool):
        if msg.data and not self._ready:
            rospy.loginfo("capture logging enabled (robot localized)")
            self._ready = True

    @property
    def ready(self) -> bool:
        return self._ready
