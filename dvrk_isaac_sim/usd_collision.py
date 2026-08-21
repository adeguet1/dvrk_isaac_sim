"""Apply collision APIs to imported URDF collision meshes in kinematic mode."""

from __future__ import annotations

import json
from pathlib import Path


def _visual_root(manifest_path: str | Path | None) -> str:
    if manifest_path is None:
        return "Geometry/world"
    try:
        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return "Geometry/world"
    collision = manifest.get("collision")
    if isinstance(collision, dict) and isinstance(collision.get("root"), str):
        return str(collision["root"])
    visual = manifest.get("visual")
    if isinstance(visual, dict) and isinstance(visual.get("root"), str):
        return str(visual["root"])
    return "Geometry/world"


def _is_collision_candidate(prim) -> bool:
    name = prim.GetName().lower()
    if name.endswith("_collision"):
        return True
    purpose = prim.GetAttribute("purpose")
    if purpose.IsValid() and purpose.HasAuthoredValueOpinion():
        if str(purpose.Get()).lower() == "guide":
            return True
    approximation = prim.GetAttribute("physics:approximation")
    return approximation.IsValid() and approximation.HasAuthoredValueOpinion()


def _apply_collision_api(prim, UsdPhysics) -> bool:
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        return False
    UsdPhysics.CollisionAPI.Apply(prim)
    return True


def apply_collision_meshes(component_name: str, manifest_path: str | Path | None = None) -> int:
    """Apply collision APIs to guide/collision prims authored by the URDF import.

    The robot remains kinematic: this only restores collision participation for
    the existing moving link hierarchy.
    """
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim stage is not available")

    visual_root = _visual_root(manifest_path)
    root_path = f"/World/{component_name}/{visual_root}"
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"USD collision root not found: {root_path}")

    applied = 0
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsActive() or not _is_collision_candidate(prim):
            continue

        geometry_targets = [item for item in Usd.PrimRange(prim) if item.IsA(UsdGeom.Gprim)]
        if geometry_targets:
            for target in geometry_targets:
                applied += int(_apply_collision_api(target, UsdPhysics))
        else:
            applied += int(_apply_collision_api(prim, UsdPhysics))
    return applied
