"""ROS 2 CRTK interface for one backend-independent PSM or ECM model."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np

from .config import RobotConfig, load_robot_config
from .kinematics import CrtkECM, CrtkPSM, CrtkComponent, Pose


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


class CrtkRosComponent:
    """ROS 2 adapter exposing CRTK topics for one kinematic component."""

    def __init__(self, node, config: RobotConfig, model: CrtkComponent):
        from crtk_msgs.msg import CartesianServo, OperatingState, StringStamped
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState

        self.node = node
        self.config = config
        self.model = model
        self._JointState = JointState
        self._PoseStamped = PoseStamped
        self._TwistStamped = TwistStamped
        self._OperatingState = OperatingState
        self._StringStamped = StringStamped
        self._frame_id = str(config.raw.get("robot", {}).get("cartesian", {}).get("reference_frame", config.parent_frame))

        self.measured_js_publisher = node.create_publisher(JointState, "measured_js", 10)
        self.measured_cp_publisher = node.create_publisher(PoseStamped, "measured_cp", 10)
        self.measured_cv_publisher = node.create_publisher(TwistStamped, "measured_cv", 10)
        self.setpoint_js_publisher = node.create_publisher(JointState, "setpoint_js", 10)
        self.operating_state_publisher = node.create_publisher(OperatingState, "operating_state", 10)
        self.state_publisher = node.create_publisher(StringStamped, "state", 10)

        node.create_subscription(JointState, "move_jp", self._move_jp_callback, 10)
        node.create_subscription(JointState, "servo_jp", self._servo_jp_callback, 10)
        node.create_subscription(PoseStamped, "move_cp", self._move_cp_callback, 10)
        node.create_subscription(CartesianServo, "servo_cp", self._servo_cp_callback, 10)

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
        try:
            self.model.move_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected move_jp: {error}")

    def _servo_jp_callback(self, message) -> None:
        try:
            self.model.servo_jp(self._positions_from_message(message))
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected servo_jp: {error}")

    def _move_cp_callback(self, message) -> None:
        try:
            result = self.model.move_cp(_pose_from_ros(message))
            if not result.success:
                self.node.get_logger().warning(f"{self.config.name} move_cp failed: {result.message}")
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected move_cp: {error}")

    def _servo_cp_callback(self, message) -> None:
        try:
            result = self.model.move_cp(_pose_from_ros(message))
            if not result.success:
                self.node.get_logger().warning(f"{self.config.name} servo_cp failed: {result.message}")
        except ValueError as error:
            self.node.get_logger().warning(f"{self.config.name} rejected servo_cp: {error}")

    def publish(self, stamp) -> None:
        joint_state = self.model.measured_js()
        pose = self.model.measured_cp()
        twist = self.model.measured_cv()

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

        measured_cv = self._TwistStamped()
        measured_cv.header.stamp = stamp
        measured_cv.header.frame_id = self._frame_id
        measured_cv.twist.linear.x, measured_cv.twist.linear.y, measured_cv.twist.linear.z = twist.linear
        measured_cv.twist.angular.x, measured_cv.twist.angular.y, measured_cv.twist.angular.z = twist.angular
        self.measured_cv_publisher.publish(measured_cv)

        operating_state = self._OperatingState()
        operating_state.header.stamp = stamp
        operating_state.header.frame_id = self._frame_id
        operating_state.state = "ENABLED"
        operating_state.is_homed = True
        operating_state.is_busy = self.model.is_busy()
        self.operating_state_publisher.publish(operating_state)

        state = self._StringStamped()
        state.header.stamp = stamp
        state.header.frame_id = self._frame_id
        state.string = "ENABLED"
        self.state_publisher.publish(state)

        setpoint = self._JointState()
        setpoint.header.stamp = stamp
        setpoint.header.frame_id = self._frame_id
        setpoint.name = list(joint_state.names)
        setpoint.position = self.model.goal_js().position.tolist()
        self.setpoint_js_publisher.publish(setpoint)


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
