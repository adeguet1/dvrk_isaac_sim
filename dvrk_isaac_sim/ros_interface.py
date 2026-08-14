"""ROS 2 CRTK interface for one backend-independent PSM or ECM model."""

from __future__ import annotations

import time
from typing import Iterable

import numpy as np

from .config import RobotConfig
from .kinematics import CRTKECM, CRTKPSM, CRTKComponent, Pose


from .cartesian_frames import (
    _VIEW_TO_OPTICAL_ROTATION,
    _compose_pose,
    _inverse_pose,
    _quaternion_matrix_xyzw,
    _relative_pose,
    _relative_twist,
    _view_pose_from_optical,
)
from .ros_messages import _pose_from_ros, _quaternion_xyzw
from .command_validation import joint_positions_from_message, jaw_position_from_message
from .ros_qos import transient_local_event_qos, transient_local_latched_qos
from .operating_state import CRTKOperatingState


class CRTKROSComponent:
    """ROS 2 adapter exposing CRTK topics for one kinematic component."""

    def __init__(self, node, config: RobotConfig, model: CRTKComponent, simulation_stamp=None):
        from crtk_msgs.msg import OperatingState, StringStamped
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        self.node = node
        self.config = config
        self.model = model
        self._JointState = JointState
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._OperatingState = OperatingState
        self._StringStamped = StringStamped
        initial_state = CRTKOperatingState.ENABLED
        self._operating_state = CRTKOperatingState(initial_state)
        self._has_jaw = config.type == "PSM"
        jaw_config = config.raw.get("robot", {}).get("jaw", {})
        self._jaw_lower = float(jaw_config.get("lower", -0.349066))
        self._jaw_upper = float(jaw_config.get("upper", 1.39626))
        self._jaw_position = 0.0
        self._jaw_velocity = 0.0
        self._frame_id = str(config.raw.get("robot", {}).get("cartesian", {}).get("reference_frame", config.parent_frame))
        self._cartesian_reference: CRTKComponent | None = None
        self._simulation_stamp = simulation_stamp
        asset = config.raw.get("robot", {}).get("asset", {})
        instrument = asset.get("instrument") if isinstance(asset, dict) else None
        self._instrument_name = str(instrument) if instrument not in (None, "") else None
        self._cartesian_reference_frame = self._frame_id
        # Servo commands can arrive faster than the Isaac update loop. Keep
        # only the newest command and solve IK from the simulation thread.
        self._pending_servo_cp = None
        self._last_published_busy = None
        self._motion_busy = False
        self._motion_start_stamp: tuple[int, int] | None = None
        self._motion_failure_pending = False
        self._last_ik_warning = None
        self._last_ik_warning_time = 0.0
        self._ik_warning_count = 0
        # Static world pose of this PSM base frame. The ECM view pose is
        # dynamic and is evaluated from ECM FK for every conversion.
        self._base_pose = Pose(
            config.base_position.copy(),
            _quaternion_matrix_xyzw(config.base_orientation_xyzw),
        )
        state_qos = transient_local_event_qos()

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
        self.info_publisher = node.create_publisher(StringStamped, "info", 10)
        self.warning_publisher = node.create_publisher(StringStamped, "warning", 10)
        self.error_publisher = node.create_publisher(StringStamped, "error", 10)
        self.tool_type_publisher = (
            node.create_publisher(String, "tool_type", transient_local_latched_qos())
            if config.type == "PSM" else None
        )

        node.create_subscription(JointState, "move_jp", self._move_jp_callback, 10)
        node.create_subscription(JointState, "servo_jp", self._servo_jp_callback, 10)
        node.create_subscription(PoseStamped, "move_cp", self._move_cp_callback, 10)
        node.create_subscription(PoseStamped, "servo_cp", self._servo_cp_callback, 1)
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
        self._publish_operating_state(self._event_stamp())
        self._publish_info("initialized; state is ENABLED")

    def publish_tool_type(self) -> None:
        """Publish the configured six-digit PSM tool type once.

        The publisher is transient-local, so subscribers that join after the
        simulation starts still receive the retained identifier.  It is not
        published during component construction because that precedes the
        Isaac timeline start.
        """
        if self.config.type != "PSM" or self._instrument_name is None:
            return
        from std_msgs.msg import String

        message = String()
        message.data = self._instrument_name
        if self.tool_type_publisher is not None:
            self.tool_type_publisher.publish(message)

    def set_simulation_stamp(self, stamp) -> None:
        """Set the simulation timestamp used by event messages.

        The Isaac runner updates this before processing each simulation step so
        state, diagnostics, and periodic CRTK messages share one time source.
        Standalone ROS-node use retains the node clock until this is called.
        """
        self._simulation_stamp = stamp

    def _event_stamp(self):
        if self._simulation_stamp is not None:
            return self._simulation_stamp
        return self.node.get_clock().now().to_msg()

    def _publish_message(self, publisher, message: str) -> None:
        event = self._StringStamped()
        event.header.stamp = self._event_stamp()
        event.header.frame_id = self._frame_id
        event.string = str(message)
        publisher.publish(event)

    def _publish_info(self, message: str) -> None:
        self._publish_message(self.info_publisher, message)

    def _publish_warning(self, message: str) -> None:
        self._publish_message(self.warning_publisher, message)

    def _publish_error(self, message: str) -> None:
        self._publish_message(self.error_publisher, message)

    def _ik_failure(self, command: str, message: str) -> None:
        text = f"{command} IK failed: {message}"
        now = time.monotonic()
        if text != self._last_ik_warning:
            self._last_ik_warning = text
            self._last_ik_warning_time = now
            self._ik_warning_count = 1
            self._publish_warning(text)
            self.node.get_logger().warning(f"{self.config.name} {text}")
            return

        self._ik_warning_count += 1
        if now - self._last_ik_warning_time >= 1.0:
            self._last_ik_warning_time = now
            self._publish_warning(
                f"{text} (repeated {self._ik_warning_count} times)"
            )
            self.node.get_logger().warning(
                f"{self.config.name} {text} "
                f"(repeated {self._ik_warning_count} times)"
            )

    def _clear_ik_warning(self) -> None:
        self._last_ik_warning = None
        self._last_ik_warning_time = 0.0
        self._ik_warning_count = 0

    def set_cartesian_reference(self, reference: CRTKComponent, frame_id: str) -> None:
        """Use a moving ECM pose as the PSM Cartesian reference frame.

        PSM Cartesian ROS topics are expressed in the current dVRK view
        frame derived from ECM optical FK. Joint topics remain local to the
        PSM and are unaffected.
        """
        if self.config.type != "PSM":
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
            self._publish_warning(
                f"rejected jaw position {value}: "
                f"expected [{self._jaw_lower}, {self._jaw_upper}] radians"
            )
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
            self._publish_warning(f"rejected GUI state command {command!r}: {error}")
            self.node.get_logger().warning(
                f"{self.config.name} rejected GUI state command {command!r}: {error}"
            )
            return False
        if self._operating_state.state in (CRTKOperatingState.DISABLED, CRTKOperatingState.FAULT):
            self.model.move_jp(self.model.measured_js().position)
        self._publish_operating_state(self._event_stamp())
        self._publish_info(f"state is now {self._operating_state.state}")
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
        self._publish_motion_edges()
        return True

    def command_cartesian_position(self, pose: Pose) -> bool:
        """Apply a GUI Cartesian target through the CRTK move_cp path."""
        if not self._motion_allowed("GUI Cartesian command"):
            return False
        result = self.model.move_cp(pose)
        if not result.success:
            self._publish_warning(f"{self.config.name} rejected GUI move_cp: {result.message}")
            self.node.get_logger().warning(
                f"{self.config.name} rejected GUI move_cp: {result.message}"
            )
            self._publish_motion_failure()
            return False
        self._publish_motion_edges()
        return True

    def _state_command_callback(self, message) -> None:
        success, error = self._operating_state.command(message.string)
        if not success:
            self._publish_warning(f"rejected state_command {message.string!r}: {error}")
            self.node.get_logger().warning(
                f"{self.config.name} rejected state_command {message.string!r}: {error}"
            )
            return
        if self._operating_state.state in (CRTKOperatingState.DISABLED, CRTKOperatingState.FAULT):
            self.model.move_jp(self.model.measured_js().position)
        self._publish_operating_state(self._event_stamp())
        state_text = f"state is now {self._operating_state.state}"
        self._publish_info(state_text)
        self.node.get_logger().info(f"{self.config.name} {state_text}")

    def _jaw_servo_jp_callback(self, message) -> None:
        if not self._motion_allowed("jaw/servo_jp"):
            return
        try:
            position = jaw_position_from_message(message)
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected jaw/servo_jp: {error}")
            return
        # The virtual instrument has no jaw dynamics. Keep a single logical jaw
        # position and report it immediately as both measured and setpoint state.
        self.command_jaw_position(position)

    def _motion_allowed(self, command: str) -> bool:
        if self._operating_state.accepts_motion:
            return True
        self.node.get_logger().debug(
            f"{self.config.name} ignored {command}: state is {self._operating_state.state}"
        )
        return False

    def _positions_from_message(self, message) -> np.ndarray:
        return joint_positions_from_message(
            message, (joint.name for joint in self.config.joints)
        )

    def _move_jp_callback(self, message) -> None:
        if not self._motion_allowed("move_jp"):
            return
        try:
            self.model.move_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected move_jp: {error}")
            return
        self._publish_motion_edges()

    def _servo_jp_callback(self, message) -> None:
        if not self._motion_allowed("servo_jp"):
            return
        try:
            self.model.servo_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected servo_jp: {error}")
            return

    def _move_cp_callback(self, message) -> None:
        if not self._motion_allowed("move_cp"):
            return
        try:
            result = self.model.move_cp(
                self._pose_from_ros(_pose_from_ros(message), message.header.frame_id)
            )
            if not result.success:
                self._ik_failure("move_cp", result.message)
                self._publish_motion_failure()
            else:
                self._clear_ik_warning()
                self._publish_motion_edges()
        except ValueError as error:
            self._publish_warning(f"rejected move_cp: {error}")
            self.node.get_logger().warning(f"{self.config.name} rejected move_cp: {error}")
            self._publish_motion_failure()

    def _servo_cp_callback(self, message) -> None:
        self._pending_servo_cp = message

    def process_pending_commands(self) -> None:
        """Apply the newest deferred servo command from the simulation loop."""
        message = self._pending_servo_cp
        self._pending_servo_cp = None
        if message is None:
            return
        if not self._motion_allowed("servo_cp"):
            return
        try:
            result = self.model.move_cp(
                self._pose_from_ros(_pose_from_ros(message), message.header.frame_id)
            )
            if not result.success:
                self._ik_failure("servo_cp", result.message)
            else:
                self._clear_ik_warning()
        except ValueError as error:
            self._publish_warning(f"rejected servo_cp: {error}")
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

    def _publish_operating_state(self, stamp, only_on_change: bool = False,
                                 busy_override: bool | None = None) -> None:
        """Publish state, optionally only when the motion busy flag changes."""
        busy = (
            busy_override
            if busy_override is not None
            else self._operating_state.accepts_motion and self._motion_busy
        )
        if only_on_change and busy == self._last_published_busy:
            return
        self._last_published_busy = busy
        operating_state = self._OperatingState()
        operating_state.header.stamp = stamp
        operating_state.header.frame_id = self._frame_id
        operating_state.state = self._operating_state.state
        operating_state.is_homed = self._operating_state.is_homed
        operating_state.is_busy = busy
        self.operating_state_publisher.publish(operating_state)

        state = self._StringStamped()
        state.header.stamp = stamp
        state.header.frame_id = self._frame_id
        state.string = self._operating_state.state
        self.state_publisher.publish(state)

    def _publish_motion_edges(self) -> None:
        """Publish a move-start edge and defer completion to a later tick."""
        stamp = self._event_stamp()
        self._motion_busy = True
        self._motion_start_stamp = (int(stamp.sec), int(stamp.nanosec))
        self._motion_failure_pending = False
        self._publish_operating_state(stamp, busy_override=True)

    def _publish_motion_failure(self) -> None:
        """Complete a rejected move handle without leaving it blocked."""
        stamp = self._event_stamp()
        self._motion_busy = True
        self._motion_start_stamp = (int(stamp.sec), int(stamp.nanosec))
        self._motion_failure_pending = True
        self._publish_operating_state(stamp, busy_override=True)

    def publish(self, stamp, valid: bool = True) -> None:
        """Publish periodic state, using zero time when simulation is paused."""
        if not valid:
            stamp = type(stamp)()
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
        # Servo commands do not affect busy. Complete only a move operation
        # that previously emitted its busy=true edge.
        stamp_key = (int(stamp.sec), int(stamp.nanosec))
        if (
            self._motion_busy
            and (self._motion_failure_pending or not self.model.is_busy())
            and self._motion_start_stamp is not None
            and stamp_key > self._motion_start_stamp
        ):
            self._motion_busy = False
            self._motion_start_stamp = None
            self._motion_failure_pending = False
            self._publish_operating_state(stamp, busy_override=False)
