"""ROS 2 CRTK interface for one backend-independent PSM or ECM model."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np

from .config import RobotConfig, load_robot_config
from .kinematics import CrtkECM, CrtkPSM, CrtkComponent, Pose


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


def _quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a rotation matrix to an ROS-order quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    result = np.asarray([x, y, z, w], dtype=float)
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


def _pose_from_ros(message) -> Pose:
    quaternion = np.array([
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ], dtype=float)
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    orientation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return Pose(
        np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z], dtype=float),
        orientation,
    )


class CrtkOperatingState:
    """Small, deterministic state machine for the CRTK operating-state API."""

    DISABLED = "DISABLED"
    ENABLED = "ENABLED"
    PAUSED = "PAUSED"
    FAULT = "FAULT"

    def __init__(self, initial_state: str = DISABLED) -> None:
        self.state = initial_state
        self.is_homed = True

    @property
    def accepts_motion(self) -> bool:
        return self.state == self.ENABLED

    def command(self, command: str) -> tuple[bool, str]:
        """Apply a CRTK ``state_command`` and return success and an error."""
        command = str(command).strip().lower()
        if command == "enable":
            if self.state == self.FAULT:
                return False, "cannot enable while in FAULT; issue clear_fault first"
            self.state = self.ENABLED
        elif command == "disable":
            self.state = self.DISABLED
        elif command == "pause":
            if self.state != self.ENABLED:
                return False, f"cannot pause from {self.state}"
            self.state = self.PAUSED
        elif command == "resume":
            if self.state != self.PAUSED:
                return False, f"cannot resume from {self.state}"
            self.state = self.ENABLED
        elif command == "home":
            self.is_homed = True
        elif command == "unhome":
            self.is_homed = False
        elif command == "fault":
            self.state = self.FAULT
        elif command in ("clear_fault", "reset"):
            if self.state != self.FAULT:
                return False, f"cannot clear fault from {self.state}"
            self.state = self.DISABLED
        else:
            return False, f"unknown state command {command!r}"
        return True, ""


class CrtkRosComponent:
    """ROS 2 adapter exposing CRTK topics for one kinematic component."""

    def __init__(self, node, config: RobotConfig, model: CrtkComponent):
        from crtk_msgs.msg import OperatingState, StringStamped
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        self.node = node
        self.config = config
        self.model = model
        self._JointState = JointState
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._OperatingState = OperatingState
        self._StringStamped = StringStamped
        initial_state = CrtkOperatingState.ENABLED
        self._operating_state = CrtkOperatingState(initial_state)
        self._has_jaw = config.type == "psm"
        jaw_config = config.raw.get("robot", {}).get("jaw", {})
        self._jaw_lower = float(jaw_config.get("lower", -0.349066))
        self._jaw_upper = float(jaw_config.get("upper", 1.39626))
        self._jaw_position = 0.0
        self._jaw_velocity = 0.0
        self._frame_id = str(config.raw.get("robot", {}).get("cartesian", {}).get("reference_frame", config.parent_frame))
        self._cartesian_reference: CrtkComponent | None = None
        self._cartesian_reference_frame = self._frame_id
        # Static world pose of this PSM base frame. The ECM view pose is
        # dynamic and is evaluated from ECM FK for every conversion.
        self._base_pose = Pose(
            config.base_position.copy(),
            _quaternion_matrix_xyzw(config.base_orientation_xyzw),
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.measured_js_publisher = node.create_publisher(JointState, "measured_js", 10)
        self.measured_cp_publisher = node.create_publisher(PoseStamped, "measured_cp", 10)
        self.setpoint_cp_publisher = node.create_publisher(PoseStamped, "setpoint_cp", 10)
        self.measured_cv_publisher = node.create_publisher(TwistStamped, "measured_cv", 10)
        if self._has_jaw:
            self.jaw_measured_js_publisher = node.create_publisher(JointState, "jaw/measured_js", 10)
            self.jaw_setpoint_js_publisher = node.create_publisher(JointState, "jaw/setpoint_js", 10)
        self.setpoint_js_publisher = node.create_publisher(JointState, "setpoint_js", 10)
        self.operating_state_publisher = node.create_publisher(OperatingState, "operating_state", state_qos)
        self.state_publisher = node.create_publisher(StringStamped, "state", state_qos)

        node.create_subscription(JointState, "move_jp", self._move_jp_callback, 10)
        node.create_subscription(JointState, "servo_jp", self._servo_jp_callback, 10)
        node.create_subscription(PoseStamped, "move_cp", self._move_cp_callback, 10)
        node.create_subscription(PoseStamped, "servo_cp", self._servo_cp_callback, 10)
        if self._has_jaw:
            self._jaw_move_subscription = node.create_subscription(
                JointState, "jaw/move_jp", self._jaw_servo_jp_callback, 10
            )
            self._jaw_servo_subscription = node.create_subscription(
                JointState, "jaw/servo_jp", self._jaw_servo_jp_callback, 10
            )
        self._state_command_subscription = node.create_subscription(
            StringStamped, "state_command", self._state_command_callback, 10
        )
        self._publish_operating_state(node.get_clock().now().to_msg())

    def set_cartesian_reference(self, reference: CrtkComponent, frame_id: str) -> None:
        """Use a moving ECM pose as the PSM Cartesian reference frame.

        PSM Cartesian ROS topics are expressed in the current dVRK view
        frame derived from ECM optical FK. Joint topics remain local to the
        PSM and are unaffected.
        """
        if self.config.type != "psm":
            return
        self._cartesian_reference = reference
        self._cartesian_reference_frame = str(frame_id)
        self._frame_id = self._cartesian_reference_frame

    def _view_to_base(self) -> Pose:
        """Return the current transform from ECM view coordinates to PSM base."""
        if self._cartesian_reference is None:
            return _inverse_pose(self._base_pose)
        # T_BV = inverse(T_WB) * T_WV. ECM FK supplies T_WC; the fixed
        # optical-to-view rotation supplies T_CV. Both are needed because
        # Cartesian teleoperation is defined in dVRK view axes, not optical axes.
        view_pose = _view_pose_from_optical(self._cartesian_reference.measured_cp())
        return _relative_pose(view_pose, self._base_pose)

    def _world_to_base(self, pose: Pose) -> Pose:
        return _relative_pose(pose, self._base_pose)

    def _base_to_world(self, pose: Pose) -> Pose:
        return _compose_pose(self._base_pose, pose)

    def _pose_for_ros(self, pose: Pose) -> Pose:
        if self._cartesian_reference is None:
            return pose
        # FK pipeline: world -> PSM base -> current ECM view.
        base_tool = self._world_to_base(pose)
        return _relative_pose(base_tool, self._view_to_base())

    def _pose_from_ros(self, pose: Pose, frame_id: str) -> Pose:
        if self._cartesian_reference is None:
            return pose
        # An explicitly world-referenced command remains useful for diagnostics
        # and preserves the single-PSM/no-ECM mode. All normal dVRK commands
        # use ECM_optical (or leave frame_id empty).
        if frame_id and frame_id == self.config.parent_frame:
            return pose
        # Command pipeline: current ECM view -> PSM base -> world, then IK.
        base_target = _compose_pose(self._view_to_base(), pose)
        return self._base_to_world(base_target)

    def _twist_for_ros(self, pose: Pose, twist):
        if self._cartesian_reference is None:
            return twist.linear, twist.angular
        linear, angular = _relative_twist(
            pose, twist, self._cartesian_reference.measured_cp(),
            self._cartesian_reference.measured_cv(),
        )
        # _relative_twist is in ECM optical axes; publish dVRK view axes.
        return (
            _VIEW_TO_OPTICAL_ROTATION.T @ linear,
            _VIEW_TO_OPTICAL_ROTATION.T @ angular,
        )

    @property
    def jaw_position(self) -> float | None:
        """Current logical PSM jaw position in radians."""
        return self._jaw_position if self._has_jaw else None

    def command_jaw_position(self, position: float) -> bool:
        """Apply a GUI/ROS-equivalent jaw target with instrument limits."""
        if not self._has_jaw or not self._motion_allowed("jaw command"):
            return False
        value = float(position)
        if not self._jaw_lower <= value <= self._jaw_upper:
            self.node.get_logger().warning(
                f"{self.config.name} rejected jaw position {value}: "
                f"expected [{self._jaw_lower}, {self._jaw_upper}] radians"
            )
            return False
        self._jaw_position = value
        self._jaw_velocity = 0.0
        return True

    @property
    def operating_state(self) -> str:
        """Current CRTK operating-state name for GUI and diagnostics."""
        return self._operating_state.state

    @property
    def is_homed(self) -> bool:
        return self._operating_state.is_homed

    def command_state(self, command: str) -> bool:
        """Apply a state command through the same path as ROS state_command."""
        success, error = self._operating_state.command(command)
        if not success:
            self.node.get_logger().warning(
                f"{self.config.name} rejected GUI state command {command!r}: {error}"
            )
            return False
        if self._operating_state.state in (CrtkOperatingState.DISABLED, CrtkOperatingState.FAULT):
            self.model.move_jp(self.model.measured_js().position)
        self._publish_operating_state(self.node.get_clock().now().to_msg())
        return True

    def command_joint_position(self, position: Iterable[float]) -> bool:
        """Apply a GUI joint target while preserving operating-state semantics."""
        if not self._motion_allowed("GUI joint command"):
            return False
        try:
            self.model.move_jp(position)
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected GUI joint command: {error}")
            return False
        return True

    def _state_command_callback(self, message) -> None:
        success, error = self._operating_state.command(message.string)
        if not success:
            self.node.get_logger().warning(
                f"{self.config.name} rejected state_command {message.string!r}: {error}"
            )
            return
        if self._operating_state.state in (CrtkOperatingState.DISABLED, CrtkOperatingState.FAULT):
            self.model.move_jp(self.model.measured_js().position)
        self._publish_operating_state(self.node.get_clock().now().to_msg())
        self.node.get_logger().info(
            f"{self.config.name} state is now {self._operating_state.state}"
        )

    def _jaw_servo_jp_callback(self, message) -> None:
        if not self._motion_allowed("jaw/servo_jp"):
            return
        if not message.position:
            self.node.get_logger().warning(f"{self.config.name} rejected jaw/servo_jp: no position")
            return
        # The virtual instrument has no jaw dynamics. Keep a single logical jaw
        # position and report it immediately as both measured and setpoint state.
        self.command_jaw_position(float(message.position[0]))

    def _motion_allowed(self, command: str) -> bool:
        if self._operating_state.accepts_motion:
            return True
        self.node.get_logger().debug(
            f"{self.config.name} ignored {command}: state is {self._operating_state.state}"
        )
        return False

    def _positions_from_message(self, message) -> np.ndarray:
        if not message.position:
            raise ValueError("joint command has no position values")
        values = np.asarray(message.position, dtype=float)
        expected_names = tuple(joint.name for joint in self.config.joints)
        if message.name:
            names = tuple(message.name)
            if set(names) != set(expected_names):
                raise ValueError(f"joint command names {names} do not match {expected_names}")
            values = np.asarray([message.position[names.index(name)] for name in expected_names], dtype=float)
        if values.shape != (len(expected_names),):
            raise ValueError("joint command has the wrong number of positions")
        return values

    def _move_jp_callback(self, message) -> None:
        if not self._motion_allowed("move_jp"):
            return
        try:
            self.model.move_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected move_jp: {error}")

    def _servo_jp_callback(self, message) -> None:
        if not self._motion_allowed("servo_jp"):
            return
        try:
            self.model.servo_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected servo_jp: {error}")

    def _move_cp_callback(self, message) -> None:
        if not self._motion_allowed("move_cp"):
            return
        try:
            result = self.model.move_cp(
                self._pose_from_ros(_pose_from_ros(message), message.header.frame_id)
            )
            if not result.success:
                self.node.get_logger().warning(f"{self.config.name} move_cp failed: {result.message}")
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected move_cp: {error}")

    def _servo_cp_callback(self, message) -> None:
        if not self._motion_allowed("servo_cp"):
            return
        try:
            result = self.model.move_cp(
                self._pose_from_ros(_pose_from_ros(message), message.header.frame_id)
            )
            if not result.success:
                self.node.get_logger().warning(f"{self.config.name} servo_cp failed: {result.message}")
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected servo_cp: {error}")

    def _publish_jaw_state(self, stamp) -> None:
        if not self._has_jaw:
            return
        for publisher in (self.jaw_measured_js_publisher, self.jaw_setpoint_js_publisher):
            jaw = self._JointState()
            jaw.header.stamp = stamp
            jaw.header.frame_id = self._frame_id
            jaw.name = ["jaw"]
            jaw.position = [self._jaw_position]
            jaw.velocity = [self._jaw_velocity]
            publisher.publish(jaw)

    def _publish_operating_state(self, stamp) -> None:
        """Publish the state event once, with transient-local durability."""
        operating_state = self._OperatingState()
        operating_state.header.stamp = stamp
        operating_state.header.frame_id = self._frame_id
        operating_state.state = self._operating_state.state
        operating_state.is_homed = self._operating_state.is_homed
        operating_state.is_busy = self._operating_state.accepts_motion and self.model.is_busy()
        self.operating_state_publisher.publish(operating_state)

        state = self._StringStamped()
        state.header.stamp = stamp
        state.header.frame_id = self._frame_id
        state.string = self._operating_state.state
        self.state_publisher.publish(state)

    def publish(self, stamp) -> None:
        joint_state = self.model.measured_js()
        world_pose = self.model.measured_cp()
        pose = self._pose_for_ros(world_pose)
        linear, angular = self._twist_for_ros(world_pose, self.model.measured_cv())

        measured_js = self._JointState()
        measured_js.header.stamp = stamp
        measured_js.header.frame_id = self._frame_id
        measured_js.name = list(joint_state.names)
        measured_js.position = joint_state.position.tolist()
        measured_js.velocity = joint_state.velocity.tolist()
        self.measured_js_publisher.publish(measured_js)

        measured_cp = self._PoseStamped()
        measured_cp.header.stamp = stamp
        measured_cp.header.frame_id = self._frame_id
        measured_cp.pose.position.x, measured_cp.pose.position.y, measured_cp.pose.position.z = pose.position
        measured_cp.pose.orientation.x, measured_cp.pose.orientation.y, measured_cp.pose.orientation.z, measured_cp.pose.orientation.w = _quaternion_xyzw(pose.orientation)
        self.measured_cp_publisher.publish(measured_cp)
        # CRTK teleoperation commonly consumes setpoint_cp from the puppet.
        # This simulator has no separate controller, so it is identical to measured_cp.
        self.setpoint_cp_publisher.publish(measured_cp)

        measured_cv = self._TwistStamped()
        measured_cv.header.stamp = stamp
        measured_cv.header.frame_id = self._frame_id
        measured_cv.twist.linear.x, measured_cv.twist.linear.y, measured_cv.twist.linear.z = linear
        measured_cv.twist.angular.x, measured_cv.twist.angular.y, measured_cv.twist.angular.z = angular
        self.measured_cv_publisher.publish(measured_cv)

        setpoint = self._JointState()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = self._frame_id
        setpoint.name = list(joint_state.names)
        setpoint.position = self.model.goal_js().position.tolist()
        self.setpoint_js_publisher.publish(setpoint)
        self._publish_jaw_state(stamp)


class CrtkRosNode:
    """ROS 2 node for one configured PSM or ECM."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node

        class _Node(Node):
            pass

        self._node = _Node("dvrk_isaac_sim")
        self._node.declare_parameter("robot_config", "")
        self._node.declare_parameter("update_rate_hz", 120.0)
        config_path = self._node.get_parameter("robot_config").get_parameter_value().string_value
        if not config_path:
            raise ValueError("robot_config ROS parameter is required")
        config = load_robot_config(Path(config_path))
        model = CrtkPSM(config) if config.type == "psm" else CrtkECM(config)
        self.component = CrtkRosComponent(self._node, config, model)
        rate = self._node.get_parameter("update_rate_hz").value
        self._node.create_timer(1.0 / float(rate), self._update)
        self._last_time = self._node.get_clock().now()

    def _update(self) -> None:
        now = self._node.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        self.component.model.step(max(0.0, dt))
        self.component.publish(now.to_msg())

    def spin(self) -> None:
        import rclpy
        rclpy.spin(self._node)

    def destroy(self) -> None:
        self._node.destroy_node()


def main() -> None:
    import rclpy

    rclpy.init()
    node = None
    try:
        node = CrtkRosNode()
        node.spin()
    finally:
        if node is not None:
            node.destroy()
        rclpy.shutdown()
