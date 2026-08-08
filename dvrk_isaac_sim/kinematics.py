"""Pure-Python CRTK-style kinematics for the virtual PSM and ECM."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .config import RobotConfig
from .urdf_kinematics import UrdfKinematicChain


@dataclass(frozen=True)
class Pose:
    position: np.ndarray
    orientation: np.ndarray


@dataclass(frozen=True)
class Twist:
    linear: np.ndarray
    angular: np.ndarray


@dataclass(frozen=True)
class JointState:
    names: tuple[str, ...]
    position: np.ndarray
    velocity: np.ndarray


@dataclass(frozen=True)
class IKResult:
    position: np.ndarray
    success: bool
    iterations: int
    position_error: float
    message: str = ""


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
    ])


def _transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


class CRTKComponent:
    """Backend-independent component exposing CRTK-style state and commands."""

    def __init__(self, config: RobotConfig, adaptor_offset: float, adaptor_rpy: tuple[float, float, float]):
        self.config = config
        self._q = config.home_position.copy()
        self._qdot = np.zeros(len(config.joints), dtype=float)
        self._target_q = self._q.copy()
        self._adaptor_offset = float(adaptor_offset)
        self._adaptor_rpy = adaptor_rpy
        self._joint_origins = self._make_joint_origins()
        self._urdf_chain = (UrdfKinematicChain(config.kinematics_manifest)
                            if config.kinematics_manifest is not None
                            and config.kinematics_manifest.is_file() else None)
        if self._urdf_chain is not None and self._urdf_chain.active_joints != tuple(joint.name for joint in config.joints):
            raise ValueError(
                f"URDF manifest joints {self._urdf_chain.active_joints} do not match "
                f"configured joints {tuple(joint.name for joint in config.joints)}"
            )

    def _make_joint_origins(self) -> tuple[tuple[np.ndarray, tuple[float, float, float]], ...]:
        origins = [(np.zeros(3), (0.0, 0.0, 0.0)) for _ in self.config.joints]
        if len(origins) >= 3:
            origins[2] = (np.array([0.0, 0.0, self._adaptor_offset]), self._adaptor_rpy)
        return tuple(origins)

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        raise NotImplementedError

    def _forward_with_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if q.shape != (len(self.config.joints),):
            raise ValueError("joint position has the wrong size")
        if self._urdf_chain is not None:
            transform, jacobian = self._urdf_chain.forward(
                q, tuple(joint.name for joint in self.config.joints)
            )
            base_rotation = _quaternion_matrix_xyzw(self.config.base_orientation_xyzw)
            base_transform = _transform(base_rotation, self.config.base_position)
            transform = base_transform @ transform
            jacobian[:3, :] = base_rotation @ jacobian[:3, :]
            jacobian[3:, :] = base_rotation @ jacobian[3:, :]
            return transform, jacobian
        transform = _transform(_quaternion_matrix_xyzw(self.config.base_orientation_xyzw), self.config.base_position)
        axes_world = []
        origins_world = []
        joint_types = []
        for index, (joint, axis, (origin_xyz, origin_rpy)) in enumerate(zip(self.config.joints, self._joint_axes(), self._joint_origins)):
            origin_transform = _transform(_rpy_matrix(*origin_rpy), origin_xyz)
            transform = transform @ origin_transform
            origins_world.append(transform[:3, 3].copy())
            axes_world.append(transform[:3, :3] @ axis)
            joint_types.append(joint.type)
            if joint.type == "revolute":
                transform = transform @ _transform(_rotation(axis, q[index]), [0.0, 0.0, 0.0])
            else:
                transform = transform @ _transform(np.eye(3), axis * q[index])

        position = transform[:3, 3]
        jacobian = np.zeros((6, len(self.config.joints)))
        for index, (axis, origin, joint_type) in enumerate(zip(axes_world, origins_world, joint_types)):
            if joint_type == "revolute":
                jacobian[:3, index] = np.cross(axis, position - origin)
                jacobian[3:, index] = axis
            else:
                jacobian[:3, index] = axis
        return transform, jacobian

    def measured_js(self) -> JointState:
        return JointState(tuple(joint.name for joint in self.config.joints), self._q.copy(), self._qdot.copy())

    def measured_cp(self, frame: str | None = None) -> Pose:
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        transform, _ = self._forward_with_jacobian(self._q)
        return Pose(transform[:3, 3].copy(), transform[:3, :3].copy())

    def measured_cv(self, frame: str | None = None) -> Twist:
        _, jacobian = self._forward_with_jacobian(self._q)
        velocity = jacobian @ self._qdot
        return Twist(velocity[:3].copy(), velocity[3:].copy())

    def goal_js(self) -> JointState:
        """Return the current move/servo joint goal using CRTK naming."""
        return JointState(tuple(joint.name for joint in self.config.joints), self._target_q.copy(), np.zeros_like(self._target_q))

    def is_busy(self) -> bool:
        return bool(np.any(np.abs(self._q - self._target_q) > 1e-9))

    def move_jp(self, joint_position: Iterable[float]) -> None:
        target = self._validate_joint_position(joint_position)
        self._target_q = target

    def servo_jp(self, joint_position: Iterable[float]) -> None:
        self.move_jp(joint_position)

    def step(self, dt: float) -> None:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        delta = self._target_q - self._q
        max_delta = np.array([joint.velocity for joint in self.config.joints]) * dt
        applied = np.clip(delta, -max_delta, max_delta)
        self._qdot = applied / dt if dt > 0.0 else np.zeros_like(applied)
        self._q += applied
        if np.allclose(self._q, self._target_q):
            self._qdot[:] = 0.0

    def compute_fk(self, q: Iterable[float] | None = None, frame: str | None = None) -> Pose:
        values = self._q if q is None else self._validate_joint_position(q)
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        transform, _ = self._forward_with_jacobian(values)
        return Pose(transform[:3, 3].copy(), transform[:3, :3].copy())

    def compute_jacobian(self, q: Iterable[float] | None = None, frame: str | None = None) -> np.ndarray:
        values = self._q if q is None else self._validate_joint_position(q)
        if frame not in (None, self.config.tool_frame, self.config.adaptor_frame):
            raise ValueError(f"unknown frame {frame!r}")
        _, jacobian = self._forward_with_jacobian(values)
        return jacobian

    def compute_ik(self, target: Pose, seed: Iterable[float] | None = None, max_iterations: int = 100) -> IKResult:
        """Solve Cartesian IK, including orientation when the chain has six DOFs."""
        q = self._q.copy() if seed is None else self._validate_joint_position(seed)
        use_orientation = len(self.config.joints) >= 6
        tolerance = 1e-5
        for iteration in range(max_iterations):
            pose = self.compute_fk(q)
            position_error = target.position - pose.position
            if use_orientation:
                # First-order world-frame rotation error. This is compatible
                # with the angular part of the spatial Jacobian.
                orientation_error = 0.5 * (
                    np.cross(pose.orientation[:, 0], target.orientation[:, 0])
                    + np.cross(pose.orientation[:, 1], target.orientation[:, 1])
                    + np.cross(pose.orientation[:, 2], target.orientation[:, 2])
                )
                error = np.concatenate((position_error, orientation_error))
                jacobian = self.compute_jacobian(q)
            else:
                error = position_error
                jacobian = self.compute_jacobian(q)[:3, :]
            error_norm = float(np.linalg.norm(error))
            if error_norm < tolerance:
                return IKResult(q, True, iteration, error_norm, "pose converged" if use_orientation else "position converged")
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-6 * np.eye(jacobian.shape[0]), error)
            q = self._clip_joint_position(q + 0.5 * step)
        pose = self.compute_fk(q)
        position_error = target.position - pose.position
        if use_orientation:
            orientation_error = 0.5 * (
                np.cross(pose.orientation[:, 0], target.orientation[:, 0])
                + np.cross(pose.orientation[:, 1], target.orientation[:, 1])
                + np.cross(pose.orientation[:, 2], target.orientation[:, 2])
            )
            error_norm = float(np.linalg.norm(np.concatenate((position_error, orientation_error))))
        else:
            error_norm = float(np.linalg.norm(position_error))
        return IKResult(q, error_norm < tolerance, max_iterations, error_norm, "pose IK did not converge" if use_orientation else "position IK did not converge")

    def move_cp(self, target: Pose) -> IKResult:
        result = self.compute_ik(target)
        if result.success:
            self.move_jp(result.position)
        return result

    def _clip_joint_position(self, position: np.ndarray) -> np.ndarray:
        lower = np.array([joint.lower for joint in self.config.joints])
        upper = np.array([joint.upper for joint in self.config.joints])
        return np.clip(position, lower, upper)

    def _validate_joint_position(self, position: Iterable[float]) -> np.ndarray:
        result = np.asarray(list(position), dtype=float)
        if result.shape != (len(self.config.joints),):
            raise ValueError("joint position has the wrong size")
        lower = np.array([joint.lower for joint in self.config.joints])
        upper = np.array([joint.upper for joint in self.config.joints])
        if np.any(result < lower) or np.any(result > upper):
            raise ValueError("joint position exceeds configured limits")
        return result


class CRTKPSM(CRTKComponent):
    """Six-DOF virtual PSM: RCM plus roll, wrist pitch, and wrist yaw."""

    def __init__(self, config: RobotConfig):
        if config.type != "PSM" or len(config.joints) != 6:
            raise ValueError("CRTKPSM requires six configured joints")
        super().__init__(config, adaptor_offset=0.4826, adaptor_rpy=(math.pi, 0.0, -math.pi / 2.0))

    def _make_joint_origins(self) -> tuple[tuple[np.ndarray, tuple[float, float, float]], ...]:
        origins = list(super()._make_joint_origins())
        # Approximate the 420006 instrument chain. The converter/visual keeps
        # the instrument-specific mesh, while this provides a useful generic
        # wrist length for Cartesian FK and measured_cp.
        if len(origins) >= 6:
            origins[3] = (np.array([0.0, 0.0, 0.4670]), (0.0, 0.0, 0.0))
            origins[4] = (np.zeros(3), (-math.pi / 2.0, -math.pi / 2.0, 0.0))
            origins[5] = (np.array([0.0107, 0.0, 0.0]), (-math.pi / 2.0, -math.pi / 2.0, 0.0))
        return tuple(origins)

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        return (
            np.array([0.0, -1.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
        )


class CRTKECM(CRTKComponent):
    """Four-DOF virtual ECM: yaw, pitch, insertion, and roll."""

    def __init__(self, config: RobotConfig):
        if config.type != "ECM" or len(config.joints) != 4:
            raise ValueError("CRTKECM requires a four-joint ECM configuration")
        super().__init__(config, adaptor_offset=0.3829, adaptor_rpy=(math.pi, 0.0, math.pi / 2.0))

    def _joint_axes(self) -> tuple[np.ndarray, ...]:
        return (
            np.array([0.0, -1.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 1.0]),
        )


def main() -> None:
    """Small smoke-test entry point installed with the ROS 2 package."""
    print("dvrk_isaac_sim kinematic core is available; launch integration is not implemented yet")
