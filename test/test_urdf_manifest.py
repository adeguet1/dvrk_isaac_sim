from pathlib import Path
import json

import numpy as np

from dvrk_isaac_sim.urdf_kinematics import UrdfKinematicChain, write_kinematics_manifest


def test_manifest_includes_collision_items(tmp_path: Path):
    urdf = tmp_path / "PSM1.urdf"
    urdf.write_text(
        """
<robot name="psm1_test">
  <link name="world"/>
  <link name="PSM1_RCM_link"/>
  <link name="PSM1_RCM_yaw_link"/>
  <link name="PSM1_tool_tip_link">
    <collision name="tip_collision">
      <origin xyz="0 0.01 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dvrk_model/meshes/tip.stl" scale="1 1 1"/>
      </geometry>
    </collision>
  </link>

  <joint name="PSM1_RCM_fixed" type="fixed">
    <parent link="world"/>
    <child link="PSM1_RCM_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="yaw" type="revolute">
    <parent link="PSM1_RCM_link"/>
    <child link="PSM1_RCM_yaw_link"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1" upper="1" velocity="1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="tool_tip" type="fixed">
    <parent link="PSM1_RCM_yaw_link"/>
    <child link="PSM1_tool_tip_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )

    manifest_path = write_kinematics_manifest(urdf, tmp_path / "kinematics.json", "PSM1")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["format"] >= 4
    assert "collision" in manifest
    collision = manifest["collision"]
    assert collision["root"] == "Geometry/world"
    assert len(collision["items"]) == 1

    item = collision["items"][0]
    assert item["name"] == "tip_collision"
    assert item["source_link"] == "PSM1_tool_tip_link"
    assert item["prim"] == "PSM1_RCM_yaw_link"
    assert item["origin_xyz"] == [0.0, 0.01, 0.0]
    assert item["geometry"]["type"] == "mesh"
    assert item["geometry"]["filename"].endswith("tip.stl")


def test_forward_all_links_matches_tip_forward(tmp_path: Path):
    urdf = tmp_path / "PSM1.urdf"
    urdf.write_text(
        """
<robot name="psm1_test">
  <link name="world"/>
  <link name="PSM1_RCM_link"/>
  <link name="PSM1_RCM_yaw_link"/>
  <link name="PSM1_tool_tip_link"/>

  <joint name="PSM1_RCM_fixed" type="fixed">
    <parent link="world"/>
    <child link="PSM1_RCM_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <joint name="yaw" type="revolute">
    <parent link="PSM1_RCM_link"/>
    <child link="PSM1_RCM_yaw_link"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1" upper="1" velocity="1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
  <joint name="tool_tip" type="fixed">
    <parent link="PSM1_RCM_yaw_link"/>
    <child link="PSM1_tool_tip_link"/>
    <origin xyz="0.01 0.0 0.2" rpy="0 0 0"/>
  </joint>
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )

    manifest_path = write_kinematics_manifest(urdf, tmp_path / "kinematics.json", "PSM1")
    chain = UrdfKinematicChain(manifest_path)
    joint_names = tuple(chain.active_joints)
    q = np.array([0.2], dtype=float)

    tip_transform, _ = chain.forward(q, joint_names)
    all_links = chain.forward_all_links(q, joint_names)
    assert chain.tip_link in all_links
    np.testing.assert_allclose(all_links[chain.tip_link], tip_transform, atol=1e-12)
