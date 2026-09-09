"""Synchronize kinematic CRTK state with manifest-described USD transforms."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class CRTKUSDVisual:
    """Apply measured joints to visual operations defined by a USD manifest."""

    def __init__(self, component_name: str, manifest_path: str | Path):
        from pxr import UsdGeom
        import omni.usd

        self._component_name = component_name
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac Sim stage is not available")
        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
        visual = manifest.get("visual")
        if not isinstance(visual, dict) or not isinstance(visual.get("joints"), dict):
            raise RuntimeError(
                f"{manifest_path}: manifest has no visual joint mapping; regenerate the USD asset"
            )

        self._UsdGeom = UsdGeom
        self._last_values: dict[str, float] = {}
        self._fabric_hierarchy = None
        self._fabric_joints = None
        root = f"/World/{component_name}/{visual.get('root', 'Geometry/world')}"
        self._visual_joints = {}
        for name, specification in visual["joints"].items():
            if not isinstance(specification, dict):
                continue
            relative_prim = str(specification.get("prim", ""))
            visual_root = str(visual.get("root", "Geometry/world")).strip("/")
            # Manifests are expected to store paths relative to visual.root.
            # Normalize an older/generated full-root path as well so a stale
            # cache cannot produce Geometry/world/Geometry/world/... paths.
            if relative_prim == visual_root:
                relative_prim = ""
            elif relative_prim.startswith(visual_root + "/"):
                relative_prim = relative_prim[len(visual_root) + 1:]
            prim_path = f"{root}/{relative_prim}" if relative_prim else root
            operation = specification.get("operation")
            axis = str(specification.get("axis", "Z"))
            motion_first = bool(specification.get("motion_before_static_transform", False))
            if operation == "rotate":
                op = self._add_rotate(prim_path, axis, motion_first)
            elif operation == "translate":
                op = self._add_translate(prim_path, axis, motion_first)
            else:
                raise RuntimeError(f"{manifest_path}: unsupported visual operation {operation!r} for {name}")
            self._visual_joints[name] = (
                op,
                float(specification.get("scale", 1.0)),
                specification.get("mimic"),
                prim_path,
                operation,
                axis,
                motion_first,
            )

    def enable_fabric(self) -> None:
        """Move runtime joint transforms from USD authoring to Fabric.

        The referenced geometry and its static transforms remain in USD.  Only
        the small local matrix which changes for each joint is written through
        IFabricHierarchy.  This avoids issuing dozens of USD change notices per
        camera frame, which otherwise makes RTX rebuild the moving scene at a
        few frames per second for a three-PSM scene.

        Call this after at least one application update, so Fabric Scene
        Delegate has mirrored the newly referenced prims and xform operations.
        """
        from pxr import UsdUtils
        import usdrt

        stage_id = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
        fabric_stage = usdrt.Usd.Stage.Attach(stage_id)
        hierarchy = usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(
            fabric_stage.GetFabricId(), fabric_stage.GetStageIdAsStageId()
        )
        if hierarchy is None:
            raise RuntimeError("Isaac Fabric Scene Delegate is not available")

        fabric_joints = {}
        for name, (_, scale, mimic, prim_path, operation, axis, motion_first) in (
            self._visual_joints.items()
        ):
            path = usdrt.Sdf.Path(prim_path)
            fabric_prim = fabric_stage.GetPrimAtPath(path)
            if not fabric_prim or not fabric_prim.IsValid():
                raise RuntimeError(f"Fabric visual link not found: {prim_path}")
            # With the CRTK operation still at its identity value, the Fabric
            # local matrix is exactly the importer-authored static transform.
            base_matrix = hierarchy.get_local_xform(path)
            fabric_joints[name] = (
                path, base_matrix, scale, mimic, operation, axis, motion_first
            )

        self._fabric_hierarchy = hierarchy
        self._fabric_joints = fabric_joints
        self._last_values.clear()

    @staticmethod
    def _fabric_motion_matrix(usdrt, operation: str, axis: str, value: float):
        matrix = usdrt.Gf.Matrix4d(1.0)
        vector = [0.0, 0.0, 0.0]
        vector["XYZ".index(axis)] = 1.0 if operation == "rotate" else value
        if operation == "rotate":
            matrix.SetRotate(
                usdrt.Gf.Rotation(
                    usdrt.Gf.Vec3d(*vector), float(np.degrees(value))
                )
            )
        else:
            matrix.SetTranslate(usdrt.Gf.Vec3d(*vector))
        return matrix

    def flush(self) -> None:
        """Recompute Fabric world matrices after a batch of local updates."""
        if self._fabric_hierarchy is not None:
            self._fabric_hierarchy.update_world_xforms()

    def _xform(self, prim_path: str):
        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"USD visual link not found: {prim_path}")
        return self._UsdGeom.Xformable(prim)

    @staticmethod
    def _place_motion_first(xform, operation) -> None:
        """Place a joint operation before importer-folded visual offsets."""
        operation_name = operation.GetOpName()
        ordered = [operation]
        ordered.extend(op for op in xform.GetOrderedXformOps()
                       if op.GetOpName() != operation_name)
        xform.SetXformOpOrder(ordered)

    def _add_rotate(self, prim_path: str, axis: str, motion_first: bool):
        xform = self._xform(prim_path)
        method = getattr(xform, f"AddRotate{axis}Op")
        operation = method(precision=self._UsdGeom.XformOp.PrecisionDouble, opSuffix="crtk")
        if motion_first:
            self._place_motion_first(xform, operation)
        return operation

    def _add_translate(self, prim_path: str, axis: str, motion_first: bool):
        # Isaac/USD translation ops are vector-valued; the manifest axis selects
        # the component that receives the joint displacement.
        xform = self._xform(prim_path)
        operation = xform.AddTranslateOp(
            precision=self._UsdGeom.XformOp.PrecisionDouble, opSuffix="crtk"
        )
        if motion_first:
            self._place_motion_first(xform, operation)
        return operation, axis

    @staticmethod
    def _set_operation(operation, value: float) -> None:
        if isinstance(operation, tuple):
            op, axis = operation
            vector = np.zeros(3, dtype=float)
            vector["XYZ".index(axis)] = value
            op.Set(tuple(float(item) for item in vector))
        else:
            operation.Set(float(np.degrees(value)))

    def update(
        self, joint_names: tuple[str, ...], joint_position: np.ndarray,
        jaw_position: float | None = None,
    ) -> None:
        values = dict(zip(joint_names, joint_position))
        joints = self._fabric_joints or self._visual_joints
        for name, specification in joints.items():
            if self._fabric_joints is not None:
                path, base_matrix, scale, mimic, operation_type, axis, motion_first = specification
                operation = None
            else:
                operation, scale, mimic, _, operation_type, axis, motion_first = specification
            if name in values:
                value = float(values[name])
            elif jaw_position is not None and isinstance(mimic, dict) and mimic.get("joint") == "jaw":
                value = float(jaw_position) * float(mimic.get("multiplier", 1.0)) + float(mimic.get("offset", 0.0))
            else:
                continue
            applied = scale * value
            # Authoring an unchanged USD attribute still invalidates Hydra and
            # can force expensive scene work. Most control ticks do not change
            # every joint, so only touch the stage when a value really moved.
            if name in self._last_values and self._last_values[name] == applied:
                continue
            if self._fabric_joints is not None:
                import usdrt

                motion_matrix = self._fabric_motion_matrix(
                    usdrt, operation_type, axis, applied
                )
                # USD evaluates its ordered operations in reverse matrix
                # multiplication order (row-vector convention).
                local_matrix = (
                    base_matrix * motion_matrix
                    if motion_first else motion_matrix * base_matrix
                )
                self._fabric_hierarchy.set_local_xform(path, local_matrix)
            else:
                self._set_operation(operation, applied)
            self._last_values[name] = applied
