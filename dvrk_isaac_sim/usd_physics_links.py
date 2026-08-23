"""Flattened, non-nested kinematic rigid bodies for instrument collision."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .urdf_kinematics import _rpy_matrix, _transform


def _candidate_collision_prim(prim, Usd, UsdGeom, item_name: str):
    item_name = item_name.lower()
    named = []
    generic = []
    for child in Usd.PrimRange(prim):
        if not child.IsActive():
            continue
        if child.IsA(UsdGeom.Gprim):
            generic.append(child)
            if item_name and item_name in child.GetName().lower():
                named.append(child)
                continue
        name = child.GetName().lower()
        if name.endswith("_collision"):
            generic.append(child)
            if item_name and item_name in name:
                named.append(child)
                continue
        purpose = child.GetAttribute("purpose")
        if purpose.IsValid() and purpose.HasAuthoredValueOpinion() and str(purpose.Get()).lower() == "guide":
            generic.append(child)
            if item_name and item_name in name:
                named.append(child)
                continue
        approximation = child.GetAttribute("physics:approximation")
        if approximation.IsValid() and approximation.HasAuthoredValueOpinion():
            generic.append(child)
            if item_name and item_name in name:
                named.append(child)
    if named:
        return named[0]
    return generic[0] if generic else None


def _transform_from_origin(origin_xyz, origin_rpy) -> np.ndarray:
    return _transform(_rpy_matrix(*[float(value) for value in origin_rpy]), [float(value) for value in origin_xyz])


def _matrix_to_quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
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
    return float(x), float(y), float(z), float(w)


class PhysicsLinkSync:
    """Synchronize flattened kinematic collision rigid bodies from URDF FK."""

    def __init__(self, component_name: str, manifest_path: str | Path, kinematic_chain):
        import omni.usd
        from pxr import PhysxSchema, Sdf, UsdGeom, UsdPhysics

        self._component_name = component_name
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac Sim stage is not available")
        self._UsdGeom = UsdGeom
        self._UsdPhysics = UsdPhysics
        self._chain = kinematic_chain
        self._link_ops: dict[str, tuple[str, np.ndarray, object, object]] = {}
        self._visual_root = "Geometry/world"

        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
        collision = manifest.get("collision") if isinstance(manifest, dict) else None
        if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
            raise RuntimeError(f"{manifest_path}: manifest has no collision.items data")
        if isinstance(collision.get("root"), str):
            self._visual_root = str(collision["root"])
        items = [item for item in collision["items"] if isinstance(item, dict)]
        if not items:
            return

        root_path = f"/World/{component_name}/PhysicsLinks"
        UsdGeom.Xform.Define(self._stage, root_path)

        links: dict[str, dict] = {}
        for item in items:
            link = str(item.get("source_link", "")).strip()
            if not link or link in links:
                continue
            links[link] = item

        for link, item in links.items():
            prim_path = f"{root_path}/{link}"
            translate_op, orient_op = self._create_flattened_link(
                prim_path, item, UsdGeom, UsdPhysics, PhysxSchema, Sdf
            )
            offset = _transform_from_origin(item.get("origin_xyz", [0.0, 0.0, 0.0]),
                                            item.get("origin_rpy", [0.0, 0.0, 0.0]))
            self._link_ops[link] = (prim_path, offset, translate_op, orient_op)

    def _create_flattened_link(self, prim_path, item, UsdGeom, UsdPhysics, PhysxSchema, Sdf):
        from pxr import Usd

        link_xform = UsdGeom.Xform.Define(self._stage, prim_path)
        link_prim = link_xform.GetPrim()

        rigid = UsdPhysics.RigidBodyAPI.Apply(link_prim)
        rigid.CreateKinematicEnabledAttr().Set(True)
        try:
            physx_rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(link_prim)
            physx_rigid.CreateEnableCCDAttr().Set(True)
        except Exception:
            # Some PhysX builds ignore or omit CCD on kinematic actors.
            pass

        xformable = UsdGeom.Xformable(link_prim)
        translate_op = xformable.AddTranslateOp(
            precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="physics"
        )
        orient_op = xformable.AddOrientOp(
            precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="physics"
        )

        source_relative = str(item.get("prim", "")).strip("/")
        if source_relative:
            source_path = f"/World/{self._component_name}/{self._visual_root}/{source_relative}"
        else:
            source_path = f"/World/{self._component_name}/{self._visual_root}"
        source_prim = self._stage.GetPrimAtPath(source_path)
        candidate = None
        if source_prim.IsValid():
            candidate = _candidate_collision_prim(
                source_prim,
                Usd,
                UsdGeom,
                str(item.get("name", "")),
            )

        if candidate is not None:
            collision_prim_path = f"{prim_path}/Collision"
            collision_prim = self._stage.DefinePrim(collision_prim_path, candidate.GetTypeName() or "Xform")
            collision_prim.GetReferences().AddInternalReference(str(candidate.GetPath()))
            if not collision_prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(collision_prim)
            geometry_targets = [child for child in Usd.PrimRange(collision_prim) if child.IsA(UsdGeom.Gprim)]
            for geometry in geometry_targets:
                if not geometry.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI.Apply(geometry)
        else:
            # Keep an explicit placeholder so missing collision manifests are visible.
            fallback = UsdGeom.Sphere.Define(self._stage, Sdf.Path(f"{prim_path}/Collision"))
            fallback.CreateRadiusAttr(0.001)
            UsdPhysics.CollisionAPI.Apply(fallback.GetPrim())

        return translate_op, orient_op

    def update(self, joint_names, joint_position, jaw_position=None):
        del jaw_position
        q = np.asarray(joint_position, dtype=float)
        poses = self._chain.forward_all_links(q, joint_names)
        for link, (_, local_offset, translate_op, orient_op) in self._link_ops.items():
            if link not in poses:
                continue
            world = poses[link] @ local_offset
            self._set_kinematic_target(translate_op, orient_op, world)

    def _set_kinematic_target(self, translate_op, orient_op, world_4x4):
        from pxr import Gf

        translation = world_4x4[:3, 3]
        rotation = world_4x4[:3, :3]
        x, y, z, w = _matrix_to_quat_xyzw(rotation)
        translate_op.Set(Gf.Vec3d(float(translation[0]), float(translation[1]), float(translation[2])))
        orient_op.Set(Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z))))