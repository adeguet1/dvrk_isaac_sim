from pathlib import Path

import numpy as np
import pytest

from dvrk_isaac_sim.config import load_robot_config
from dvrk_isaac_sim.kinematics import CrtkECM, CrtkPSM, Pose


ROOT = Path(__file__).parents[1]


def test_psm_home_pose_and_jacobian():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    pose = robot.measured_cp()
    assert pose.position.shape == (3,)
    assert pose.orientation.shape == (3, 3)
    np.testing.assert_allclose(pose.orientation @ pose.orientation.T, np.eye(3), atol=1e-12)
    assert robot.compute_jacobian().shape == (6, 3)


def test_psm_move_jp_respects_velocity_limit():
    robot = CrtkPSM(load_robot_config(ROOT / "config/PSM1.yaml"))
    robot.move_jp([0.5, 0.2, 0.1])
    robot.step(0.1)
    np.testing.assert_allclose(robot.measured_js().position, [0.1, 0.1, 0.04])
    np.testing.assert_allclose(robot.measured_js().velocity, [1.0, 1.0, 0.4])


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
    target_q = np.array([0.2, -0.1, 0.08])
    target = robot.compute_fk(target_q)
    result = robot.compute_ik(Pose(target.position, target.orientation), seed=[0.0, 0.0, 0.0])
    assert result.success
    np.testing.assert_allclose(robot.compute_fk(result.position).position, target.position, atol=1e-4)
