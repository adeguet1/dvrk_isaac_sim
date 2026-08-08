"""QoS profiles used by the CRTK ROS adapter."""

from __future__ import annotations


def transient_local_event_qos():
    """QoS for latched operating-state and state-event topics."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos
