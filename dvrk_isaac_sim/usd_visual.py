"""Synchronize the kinematic CRTK model with visual-only USD link transforms."""

from __future__ import annotations

import numpy as np


class CrtkUsdVisual:
    """Apply measured joint positions to a referenced virtual dVRK asset."""

    def __init__(self, component_name: str):
        from pxr import UsdGeom
        import omni.usd

        self._component_name = component_name
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac Sim stage is not available")
        self._UsdGeom = UsdGeom
        root = f"/World/{component_name}/Geometry/world"
        prefix = f"{root}/{component_name}_RCM_yaw_link"
        self._ops = {
            "yaw": self._add_rotate(prefix, "Y"),
            "pitch": self._add_rotate(f"{prefix}/{component_name}_RCM_pitch_link", "X"),
            "insertion": self._add_translate(
                f"{prefix}/{component_name}_RCM_pitch_link/{component_name}_adaptor_link"
            ),
        }
        roll_path = (
            f"{prefix}/{component_name}_RCM_pitch_link/"
            f"{component_name}_adaptor_link/{component_name}_roll_link"
        )
        if self._stage.GetPrimAtPath(roll_path).IsValid():
            self._ops["roll"] = self._add_rotate(roll_path, "Z")
        wrist_pitch_path = f"{roll_path}/{component_name}_wrist_pitch_link"
        if self._stage.GetPrimAtPath(wrist_pitch_path).IsValid():
            self._ops["wrist_pitch"] = self._add_rotate(wrist_pitch_path, "Z")
        wrist_yaw_path = f"{wrist_pitch_path}/{component_name}_wrist_yaw_link"
        if self._stage.GetPrimAtPath(wrist_yaw_path).IsValid():
            self._ops["wrist_yaw"] = self._add_rotate(wrist_yaw_path, "Z")

    def _xform(self, prim_path: str):
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"USD visual link not found: {prim_path}")
        return self._UsdGeom.Xformable(prim)

    def _add_rotate(self, prim_path: str, axis: str):
        method = getattr(self._xform(prim_path), f"AddRotate{axis}Op")
        return method(precision=self._UsdGeom.XformOp.PrecisionDouble, opSuffix="crtk")

    def _add_translate(self, prim_path: str):
        return self._xform(prim_path).AddTranslateOp(
            precision=self._UsdGeom.XformOp.PrecisionDouble,
            opSuffix="crtk",
        )

    def update(self, joint_names: tuple[str, ...], joint_position: np.ndarray) -> None:
        values = dict(zip(joint_names, joint_position))
        if "yaw" in values:
            self._ops["yaw"].Set(float(-np.degrees(values["yaw"])))
        if "pitch" in values:
            self._ops["pitch"].Set(float(-np.degrees(values["pitch"])))
        if "insertion" in values:
            self._ops["insertion"].Set((0.0, 0.0, float(values["insertion"])))
        if "roll" in values and "roll" in self._ops:
            self._ops["roll"].Set(float(np.degrees(values["roll"])))
        if "wrist_pitch" in values and "wrist_pitch" in self._ops:
            self._ops["wrist_pitch"].Set(float(np.degrees(values["wrist_pitch"])))
        if "wrist_yaw" in values and "wrist_yaw" in self._ops:
            self._ops["wrist_yaw"].Set(float(np.degrees(values["wrist_yaw"])))
