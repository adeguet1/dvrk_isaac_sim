from pathlib import Path

import numpy as np
import pytest

from dvrk_isaac_sim.config import load_robot_config
from dvrk_isaac_sim.kinematics import CrtkECM, CrtkPSM, Pose
from dvrk_isaac_sim.ros_interface import (
    CrtkOperatingState,
    _VIEW_TO_OPTICAL_ROTATION,
    _view_pose_from_optical,
)


ROOT = Path(__file__).parents[1]


def test_psm_home_pose_and_jacobian():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    pose = robot.measured_cp()
    assert pose.position.shape == (3,)
    assert pose.orientation.shape == (3, 3)
    np.testing.assert_allclose(pose.orientation @ pose.orientation.T, np.eye(3), atol=1e-12)
    assert robot.compute_jacobian().shape == (6, 6)


def test_psm_move_jp_respects_velocity_limit():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    robot.move_jp([0.5, 0.2, 0.2, 0.1, 0.1, 0.1])
    robot.step(0.1)
    np.testing.assert_allclose(robot.measured_js().position, [0.1, 0.1, 0.16, 0.04, 0.04, 0.04])
    np.testing.assert_allclose(robot.measured_js().velocity, [1.0, 1.0, 0.4, 0.4, 0.4, 0.4])


def test_ecm_has_four_joints():
    robot = CrtkECM(load_robot_config(ROOT / "config/ECM.yaml"))
    assert robot.measured_js().names == ("yaw", "pitch", "insertion", "roll")
    assert robot.compute_jacobian().shape == (6, 4)


def test_joint_limits_are_rejected():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    with pytest.raises(ValueError):
        robot.move_jp([2.0, 0.0, 0.0])


def test_position_ik_reaches_a_nearby_target():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    target_q = np.array([0.2, -0.1, 0.08, 0.1, -0.1, 0.1])
    target = robot.compute_fk(target_q)
    result = robot.compute_ik(Pose(target.position, target.orientation), seed=[0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    assert result.success
    np.testing.assert_allclose(robot.compute_fk(result.position).position, target.position, atol=1e-4)


def test_operating_state_machine():
    state = CrtkOperatingState()
    assert state.state == CrtkOperatingState.DISABLED
    assert state.is_homed
    assert not state.accepts_motion

    assert state.command("enable")[0]
    assert state.state == CrtkOperatingState.ENABLED
    assert state.accepts_motion
    assert state.command("pause")[0]
    assert not state.accepts_motion
    assert state.command("resume")[0]
    assert state.command("unhome")[0]
    assert not state.is_homed
    assert state.command("home")[0]
    assert state.is_homed

    assert state.command("fault")[0]
    assert not state.command("enable")[0]
    assert state.command("clear_fault")[0]
    assert state.state == CrtkOperatingState.DISABLED
    assert not state.command("not-a-command")[0]


def test_psm_pose_ik_reaches_orientation():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    target = robot.compute_fk([0.15, -0.1, 0.1, 0.2, -0.15, 0.1])
    result = robot.compute_ik(target, seed=[0.0, 0.0, 0.08, 0.0, 0.0, 0.0])
    assert result.success
    solved = robot.compute_fk(result.position)
    np.testing.assert_allclose(solved.position, target.position, atol=1e-4)
    np.testing.assert_allclose(solved.orientation, target.orientation, atol=1e-4)


def test_dvrk_view_axes_are_derived_from_ecm_optical_axes():
    optical = Pose(np.zeros(3), np.eye(3))
    view = _view_pose_from_optical(optical)
    # dVRK view: X-left, Y-up, Z-away. Isaac camera optical: X-forward,
    # Y-left, Z-up. Therefore V(X,Y,Z) maps to C(Y,Z,X).
    np.testing.assert_allclose(view.orientation, _VIEW_TO_OPTICAL_ROTATION)
    np.testing.assert_allclose(view.orientation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(view.orientation @ [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])
    np.testing.assert_allclose(view.orientation @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
