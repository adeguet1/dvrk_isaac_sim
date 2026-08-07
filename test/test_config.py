from pathlib import Path

import numpy as np

from dvrk_isaac_sim.config import load_robot_config


ROOT = Path(__file__).parents[1]


def test_psm_config_loads():
    config = load_robot_config(ROOT / "config/PSM1.yaml")
    assert config.name == "PSM1"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0])


def test_ecm_config_loads():
    config = load_robot_config(ROOT / "config/ECM.yaml")
    assert config.name == "ECM"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.08, 0.0])


def test_psm_instances_include_shared_defaults():
    for name in ("PSM1", "PSM2", "PSM3"):
        config = load_robot_config(ROOT / "config" / f"{name}.yaml")
        assert config.name == name
        assert len(config.joints) == 6
        assert all(joint.velocity == (0.4 if joint.name == "insertion" else 1.0)
                   for joint in config.joints)
