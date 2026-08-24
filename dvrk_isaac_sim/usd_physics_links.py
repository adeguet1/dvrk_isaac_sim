"""Flattened, non-nested kinematic rigid bodies for instrument collision."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .urdf_kinematics import _rpy_matrix, _transform


def _usd_matrix_to_numpy_column_major(matrix) -> np.ndarray:
    return np.array(matrix, dtype=float).T


def _relative_offset_from_stage_transforms(world_link, world_candidate) -> np.ndarray:
    link_column_major = _usd_matrix_to_numpy_column_major(world_link)
    candidate_column_major = _usd_matrix_to_numpy_column_major(world_candidate)
    return np.linalg.inv(link_column_major) @ candidate_column_major


def _collision_name_hints(item: dict) -> tuple[str, ...]:
    hints: list[str] = []
    item_name = str(item.get("name", "")).strip().lower()
    if item_name:
        hints.append(item_name)
    source_link = str(item.get("source_link", "")).strip().lower()
    if source_link:
        hints.append(source_link)
    geometry = item.get("geometry")
    if isinstance(geometry, dict):
        filename = str(geometry.get("filename", "")).strip().lower()
        if filename:
            hints.append(Path(filename).stem)
    return tuple(dict.fromkeys(hint for hint in hints if hint))


def _candidate_collision_prim(prim, Usd, UsdGeom, item: dict, blocked_paths: tuple[str, ...] = ()):
    hints = _collision_name_hints(item)
    source_path = str(prim.GetPath())
    named = []
    generic = []
    for child in Usd.PrimRange(prim):
        child_path = str(child.GetPath())
        if any(child_path == blocked or child_path.startswith(blocked + "/") for blocked in blocked_paths):
            continue
        if not child.IsActive():
            continue
        depth = child_path.count("/") - source_path.count("/")
        if child.IsA(UsdGeom.Gprim):
            generic.append((depth, child_path, child))
            if hints and any(hint in child.GetName().lower() or hint in child_path.lower() for hint in hints):
                named.append((depth, child_path, child))
                continue
        name = child.GetName().lower()
        if name.endswith("_collision"):
            generic.append((depth, child_path, child))
            if hints and any(hint in name or hint in child_path.lower() for hint in hints):
                named.append((depth, child_path, child))
                continue
        purpose = child.GetAttribute("purpose")
        if purpose.IsValid() and purpose.HasAuthoredValueOpinion() and str(purpose.Get()).lower() == "guide":
            generic.append((depth, child_path, child))
            if hints and any(hint in name or hint in child_path.lower() for hint in hints):
                named.append((depth, child_path, child))
                continue
        approximation = child.GetAttribute("physics:approximation")
        if approximation.IsValid() and approximation.HasAuthoredValueOpinion():
            generic.append((depth, child_path, child))
            if hints and any(hint in name or hint in child_path.lower() for hint in hints):
                named.append((depth, child_path, child))
    if len(named) > 1:
        named.sort(key=lambda entry: (entry[0], entry[1]))
        print(
            f"[usd_physics_links] ambiguous collision match for {str(item.get('name', ''))!r}: "
            f"{[path for _, path, _ in named]} - using {named[0][2].GetPath()}"
        )
    if named:
        named.sort(key=lambda entry: (entry[0], entry[1]))
        return named[0][2]
    if generic:
        generic.sort(key=lambda entry: (entry[0], entry[1]))
        return generic[0][2]
    return None


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


def _require_collision_candidate(candidate, item: dict, source_path: str):
    if candidate is not None:
        return candidate
    link_name = str(item.get("source_link", "")).strip() or str(item.get("name", "")).strip()
    raise RuntimeError(
        f"No collision prim matched link {link_name!r} below {source_path}; "
        "cannot create flattened collision body"
    )


class PhysicsLinkSync:
    """Synchronize flattened kinematic collision rigid bodies from URDF FK."""

    def __init__(self, component_name: str, manifest_path: str | Path, kinematic_chain):
        import omni.usd
        from pxr import PhysxSchema, UsdGeom, UsdPhysics

        self._component_name = component_name
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Isaac Sim stage is not available")
        self._UsdGeom = UsdGeom
        self._UsdPhysics = UsdPhysics
        self._chain = kinematic_chain
        self._link_ops: dict[str, tuple[str, np.ndarray, object, object]] = {}
        self._link_source_paths: dict[str, str] = {}
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
            source_relative = str(item.get("prim", "")).strip("/")
            if source_relative:
                self._link_source_paths[link] = f"/World/{component_name}/{self._visual_root}/{source_relative}"
            else:
                self._link_source_paths[link] = f"/World/{component_name}/{self._visual_root}"

        for link, item in links.items():
            prim_path = f"{root_path}/{link}"
            translate_op, orient_op, offset = self._create_flattened_link(
                prim_path, item, UsdGeom, UsdPhysics, PhysxSchema
            )
            self._link_ops[link] = (prim_path, offset, translate_op, orient_op)

    def _create_flattened_link(self, prim_path, item, UsdGeom, UsdPhysics, PhysxSchema):
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
            blocked_paths = tuple(
                other_source_path
                for other_link, other_source_path in self._link_source_paths.items()
                if other_link != str(item.get("source_link", "")).strip()
                and other_source_path.startswith(source_path + "/")
            )
            candidate = _candidate_collision_prim(
                source_prim,
                Usd,
                UsdGeom,
                item,
                blocked_paths,
            )
        candidate = _require_collision_candidate(candidate, item, source_path)

        offset = _transform_from_origin(
            item.get("origin_xyz", [0.0, 0.0, 0.0]),
            item.get("origin_rpy", [0.0, 0.0, 0.0]),
        )

        world_link = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        candidate_parent = candidate.GetParent()
        if candidate_parent.IsValid():
            world_target_parent = UsdGeom.Xformable(candidate_parent).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        else:
            world_target_parent = world_link
        offset = _relative_offset_from_stage_transforms(world_link, world_target_parent)

        collision_prim_path = f"{prim_path}/Collision"
        collision_prim = self._stage.DefinePrim(collision_prim_path, candidate.GetTypeName() or "Xform")
        collision_prim.GetReferences().AddInternalReference(str(candidate.GetPath()))
        if not collision_prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(collision_prim)
        geometry_targets = [child for child in Usd.PrimRange(collision_prim) if child.IsA(UsdGeom.Gprim)]
        for geometry in geometry_targets:
            if not geometry.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(geometry)

        return translate_op, orient_op, offset

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