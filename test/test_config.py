from pathlib import Path

import numpy as np

from dvrk_isaac_sim.config import load_robot_config
from dvrk_isaac_sim.scene import available_scene_names, load_scene, load_simulator_config, resolve_scene_path


ROOT = Path(__file__).parents[1]


def test_psm_config_loads():
    config = load_robot_config(ROOT / "share/arms/PSM1.yaml")
    assert config.name == "PSM1"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll", "wrist_pitch", "wrist_yaw"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.12, 0.0, 0.0, 0.0])


def test_ecm_config_loads():
    config = load_robot_config(ROOT / "share/arms/ECM.yaml")
    assert config.name == "ECM"
    assert [joint.name for joint in config.joints] == ["yaw", "pitch", "insertion", "roll"]
    np.testing.assert_allclose(config.home_position, [0.0, 0.0, 0.08, 0.0])


def test_psm_instances_include_shared_defaults():
    for name in ("PSM1", "PSM2", "PSM3"):
        config = load_robot_config(ROOT / "share" / "arms" / f"{name}.yaml")
        assert config.name == name
        assert len(config.joints) == 6
        assert all(joint.velocity == (0.4 if joint.name == "insertion" else 1.0)
                   for joint in config.joints)


def test_scene_resolution_and_scene_owned_variants():
    config_path = ROOT / "share" / "isaac_sim.yaml"
    assert "PSM2_420093.yaml" in available_scene_names(config_path)
    scene_path = resolve_scene_path(config_path, "PSM2_420093.yaml")
    scene = load_scene(scene_path)
    assert scene.camera.mode == "mono"
    assert scene.camera.as_dict().get("transport") == "rtsp_and_h264"
    assert [(robot.name, robot.instrument, robot.endoscope) for robot in scene.robots] == [
        ("PSM2", "420093", None),
        ("ECM", None, "Si_straight"),
    ]
    assert resolve_scene_path(config_path, scene_path) == scene_path


def test_simulator_config_is_typed_and_scene_free_by_default():
    config = load_simulator_config(ROOT / "share" / "isaac_sim.yaml")
    assert config.renderer == "RaytracedLighting"
    assert config.simulation_rate_hz == 120.0
    assert config.headless is False
    assert config.scene is None
    assert config.generated_dir.is_absolute()
