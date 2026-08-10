"""QoS profiles used by the CRTK ROS adapter."""

from __future__ import annotations


def transient_local_event_qos():
    """QoS for latched operating-state and state-event topics.

    Keep the same depth as the CRTK Python client's event subscription so a
    busy-start/busy-end pair is retained for a late-joining subscriber.
    """
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos
