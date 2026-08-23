#!/usr/bin/env python3
"""Print flattened collision/link offsets from the live Isaac Sim stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _load_items(manifest_path: Path) -> tuple[str, list[dict]]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    collision = manifest.get("collision") if isinstance(manifest, dict) else None
    if not isinstance(collision, dict) or not isinstance(collision.get("items"), list):
        raise RuntimeError(f"{manifest_path}: manifest has no collision.items data")
    return str(collision.get("root", "Geometry/world")), [item for item in collision["items"] if isinstance(item, dict)]


def _link_source_paths(component: str, visual_root: str, items: list[dict]) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in items:
        link = str(item.get("source_link", "")).strip()
        if not link or link in result:
            continue
        source_relative = str(item.get("prim", "")).strip("/")
        if source_relative:
            result[link] = f"/World/{component}/{visual_root}/{source_relative}"
        else:
            result[link] = f"/World/{component}/{visual_root}"
    return result


def report(stage, component: str, visual_root: str, item: dict, source_paths: dict[str, str]):
    import dvrk_isaac_sim.usd_physics_links as usd_physics_links
    from dvrk_isaac_sim.usd_physics_links import _candidate_collision_prim
    from pxr import UsdGeom  # type: ignore[import-not-found]
    from pxr import Usd  # type: ignore[import-not-found]

    source_link = str(item.get("source_link", ""))
    link_prim_path = source_paths.get(source_link, f"/World/{component}/{visual_root}")
    source = stage.GetPrimAtPath(link_prim_path)
    if not source.IsValid():
        print(f"[{source_link}] MISSING link prim: {link_prim_path}")
        return

    blocked_paths = tuple(
        other_source_path
        for other_link, other_source_path in source_paths.items()
        if other_link != source_link and other_source_path.startswith(link_prim_path + "/")
    )
    candidate = _candidate_collision_prim(source, Usd, UsdGeom, item, blocked_paths)
    world_link = UsdGeom.Xformable(source).ComputeLocalToWorldTransform()

    flattened_collision_path = f"/World/{component}/PhysicsLinks/{source_link}/Collision"
    flattened_collision = stage.GetPrimAtPath(flattened_collision_path)
    if not flattened_collision.IsValid():
        print(f"[{source_link}] no flattened collision prim yet")
        return

    world_flat = UsdGeom.Xformable(flattened_collision).ComputeLocalToWorldTransform()
    print(source_link, flattened_collision.GetPath())
    print("  module path:     ", usd_physics_links.__file__)
    print("  manifest name:   ", item.get("name"))
    print("  source prim:     ", link_prim_path)
    print("  blocked paths:   ", blocked_paths if blocked_paths else "<none>")
    print("  nested candidate:", candidate.GetPath() if candidate is not None else "<none>")
    print("  link world:      ", world_link.ExtractTranslation())
    print("  flat collision:  ", world_flat.ExtractTranslation())
    print("  URDF origin used:", item.get("origin_xyz"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("component", help="Component name under /World, for example PSM1")
    parser.add_argument("manifest", type=Path, help="Kinematics manifest JSON used by PhysicsLinkSync")
    args = parser.parse_args()

    import omni.usd  # type: ignore[import-not-found]

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim stage is not available")

    visual_root, items = _load_items(args.manifest)
    source_paths = _link_source_paths(args.component, visual_root, items)
    for item in items:
        report(stage, args.component, visual_root, item, source_paths)


if __name__ == "__main__":
    main()