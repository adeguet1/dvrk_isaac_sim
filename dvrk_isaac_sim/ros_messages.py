"""Pure-Python conversions between ROS-shaped messages and CRTK values."""

from __future__ import annotations

import numpy as np

from .kinematics import Pose
from .rotations import (
    quaternion_matrix_xyzw,
    rotation_to_quaternion_xyzw as _quaternion_xyzw,
)



def _pose_from_ros(message) -> Pose:
    quaternion = np.array([
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ], dtype=float)
    orientation = quaternion_matrix_xyzw(quaternion)
    return Pose(
        np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z], dtype=float),
        orientation,
    )


