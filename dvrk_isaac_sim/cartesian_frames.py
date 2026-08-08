"""Pure-Python CRTK Cartesian frame and message conversions."""

from __future__ import annotations

import numpy as np

from .kinematics import Pose


# Isaac's world-camera axes are +X forward, +Y left, +Z up.  dVRK
# teleoperation uses X left, Y up, Z away from the operator.  This maps
# dVRK view coordinates into the ECM optical/camera coordinates.
_VIEW_TO_OPTICAL_ROTATION = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def _compose_pose(first: Pose, second: Pose) -> Pose:
    """Compose two poses represented as rotation/translation matrices."""
    return Pose(
        first.position + first.orientation @ second.position,
        first.orientation @ second.orientation,
    )


def _inverse_pose(pose: Pose) -> Pose:
    """Return the inverse of a rigid pose."""
    rotation = pose.orientation.T
    return Pose(-rotation @ pose.position, rotation)


def _relative_pose(pose: Pose, reference: Pose) -> Pose:
    """Express ``pose`` in the coordinate frame represented by ``reference``."""
    return _compose_pose(_inverse_pose(reference), pose)


def _view_pose_from_optical(optical_pose: Pose) -> Pose:
    """Return the dVRK view pose derived from the current ECM optical FK."""
    return _compose_pose(
        optical_pose, Pose(np.zeros(3), _VIEW_TO_OPTICAL_ROTATION)
    )


def _quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _relative_twist(pose: Pose, twist, reference: Pose, reference_twist) -> tuple[np.ndarray, np.ndarray]:
    """Express a world twist in a moving reference frame."""
    delta = pose.position - reference.position
    linear = twist.linear - reference_twist.linear - np.cross(reference_twist.angular, delta)
    angular = twist.angular - reference_twist.angular
    rotation = reference.orientation.T
    return rotation @ linear, rotation @ angular


