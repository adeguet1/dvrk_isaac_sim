#!/usr/bin/env python3
"""Lightweight cube-contact recorder for a running Isaac Sim scene.

Run this inside an already-open Isaac Sim session (Script Editor or Python
console). It does not start a new SimulationApp.

Purpose: identify which tool collision prim is involved when the cube is
touched/moved or when a likely pass-through event occurs.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np


DEFAULT_COMPONENT = "PSM1"
DEFAULT_PROP = "test_cube"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "contact_record.csv"
DEFAULT_DURATION_S = 20.0
DEFAULT_RATE_HZ = 10.0
DEFAULT_PROP_MOVE_THRESHOLD_M = 0.0015
DEFAULT_TOUCH_DISTANCE_M = 0.008


def _component_root(name: str) -> str:
    return f"/World/{name}"


def _prop_root(name: str) -> str:
    return f"/World/Environment/{name}"


def _wait_for_stage(timeout_s: float = 10.0):
    import omni.usd

    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            return stage
        if time.monotonic() >= deadline:
            raise RuntimeError("Isaac Sim stage is not available; open the scene before running this script")
        time.sleep(0.05)


def _prim_world_transform(stage, prim_path: str):
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    return UsdGeom.XformCache().GetLocalToWorldTransform(prim)


def _prim_translation(transform) -> tuple[float, float, float]:
    if transform is None:
        return (float("nan"), float("nan"), float("nan"))
    translation = transform.ExtractTranslation()
    return tuple(float(value) for value in translation)


def _prim_bounds(stage, prim_path: str) -> dict[str, tuple[float, ...]]:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        nan_triplet = (float("nan"),) * 3
        return {"range_min": nan_triplet, "range_max": nan_triplet, "extent": nan_triplet}

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
    box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    range_min = box.GetMin()
    range_max = box.GetMax()
    extent = range_max - range_min
    return {
        "range_min": tuple(float(value) for value in range_min),
        "range_max": tuple(float(value) for value in range_max),
        "extent": tuple(float(value) for value in extent),
    }


def _collision_approximation(prim) -> str:
    approximation = prim.GetAttribute("physics:approximation")
    if approximation.IsValid() and approximation.HasAuthoredValueOpinion():
        try:
            return str(approximation.Get())
        except Exception:
            return "<unreadable>"
    return "<unset>"


def _is_collision_candidate(prim) -> bool:
    from pxr import UsdPhysics

    name = prim.GetName().lower()
    if name.endswith("_collision"):
        return True
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        return True
    approximation = prim.GetAttribute("physics:approximation")
    return approximation.IsValid() and approximation.HasAuthoredValueOpinion()


def _iter_collision_prims(stage, root_path: str):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return []
    return [prim for prim in Usd.PrimRange(root) if prim.IsActive() and _is_collision_candidate(prim)]


def _pick_nearest_collision(stage, component_name: str, tool_path: str):
    tool_transform = _prim_world_transform(stage, tool_path)
    tool_position = np.asarray(_prim_translation(tool_transform), dtype=float)
    best = None
    for prim in _iter_collision_prims(stage, _component_root(component_name)):
        prim_path = str(prim.GetPath())
        collision_transform = _prim_world_transform(stage, prim_path)
        collision_position = np.asarray(_prim_translation(collision_transform), dtype=float)
        distance = float(np.linalg.norm(tool_position - collision_position))
        if best is None or distance < best["distance"]:
            best = {
                "prim": prim_path,
                "distance": distance,
                "translation": tuple(float(v) for v in collision_position),
                "approximation": _collision_approximation(prim),
            }
    if best is None:
        return {
            "prim": "",
            "distance": float("nan"),
            "translation": (float("nan"), float("nan"), float("nan")),
            "approximation": "<none>",
        }
    return best


def _aabb_overlap(bounds_a: dict[str, tuple[float, ...]], bounds_b: dict[str, tuple[float, ...]]) -> bool:
    a_min = np.asarray(bounds_a["range_min"], dtype=float)
    a_max = np.asarray(bounds_a["range_max"], dtype=float)
    b_min = np.asarray(bounds_b["range_min"], dtype=float)
    b_max = np.asarray(bounds_b["range_max"], dtype=float)
    return bool(np.all(a_min <= b_max) and np.all(b_min <= a_max))


def _aabb_distance(bounds_a: dict[str, tuple[float, ...]], bounds_b: dict[str, tuple[float, ...]]) -> float:
    a_min = np.asarray(bounds_a["range_min"], dtype=float)
    a_max = np.asarray(bounds_a["range_max"], dtype=float)
    b_min = np.asarray(bounds_b["range_min"], dtype=float)
    b_max = np.asarray(bounds_b["range_max"], dtype=float)
    gap = np.maximum(0.0, np.maximum(a_min - b_max, b_min - a_max))
    return float(np.linalg.norm(gap))


def _tool_anchor(stage, component_name: str, candidates: tuple[str, ...]) -> str:
    for candidate in candidates:
        prim_path = f"/World/{component_name}/{candidate}"
        if stage.GetPrimAtPath(prim_path).IsValid():
            return prim_path
    return _component_root(component_name)


def _inside_aabb(point: np.ndarray, bounds_min: tuple[float, ...], bounds_max: tuple[float, ...]) -> bool:
    if np.isnan(point).any():
        return False
    lower = np.asarray(bounds_min, dtype=float)
    upper = np.asarray(bounds_max, dtype=float)
    return bool(np.all(point >= lower) and np.all(point <= upper))


def _write_header(path: Path, fieldnames: list[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fieldnames)
    writer.writeheader()
    return handle, writer


def _close_writer(handle) -> None:
    if handle is not None:
        handle.flush()
        handle.close()


def _collect_collision_prims(stage, component_name: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for prim in _iter_collision_prims(stage, _component_root(component_name)):
        rows.append(
            {
                "prim": str(prim.GetPath()),
                "approximation": _collision_approximation(prim),
            }
        )
    return rows


def _pick_contact_part(stage, cube_bounds: dict[str, tuple[float, ...]], collision_rows: list[dict[str, str]]):
    best = None
    for collision in collision_rows:
        prim_path = collision["prim"]
        part_bounds = _prim_bounds(stage, prim_path)
        overlap = _aabb_overlap(part_bounds, cube_bounds)
        distance = _aabb_distance(part_bounds, cube_bounds)
        rank = (0 if overlap else 1, distance)
        if best is None or rank < best["rank"]:
            best = {
                "prim": prim_path,
                "approximation": collision["approximation"],
                "distance": distance,
                "overlap": overlap,
                "rank": rank,
            }
    if best is None:
        return {
            "prim": "",
            "approximation": "<none>",
            "distance": float("nan"),
            "overlap": False,
            "rank": (1, float("inf")),
        }
    return best


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--component", default=DEFAULT_COMPONENT, help="robot component to monitor")
    parser.add_argument("--prop", default=DEFAULT_PROP, help="single prop to track (default: test_cube)")
    parser.add_argument("--csv", type=Path, default=DEFAULT_OUTPUT, help="recording CSV path")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="recording duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="sampling rate")
    parser.add_argument("--stage-wait-timeout", type=float, default=10.0, help="seconds to wait for a live stage")
    parser.add_argument("--prop-move-threshold", type=float, default=DEFAULT_PROP_MOVE_THRESHOLD_M, help="cube movement threshold in meters")
    parser.add_argument("--touch-distance", type=float, default=DEFAULT_TOUCH_DISTANCE_M, help="AABB distance threshold in meters")
    parser.add_argument(
        "--tool-anchor",
        nargs="*",
        default=("tool", "tool_tip", "saw", "instrument_tip", "tip"),
        help="candidate prim names, in order, used as the tool reference point",
    )
    args, _unknown = parser.parse_known_args(argv)

    stage = _wait_for_stage(args.stage_wait_timeout)
    tool_path = _tool_anchor(stage, args.component, tuple(args.tool_anchor))
    cube_path = _prop_root(args.prop)
    collision_rows = _collect_collision_prims(stage, args.component)
    if not stage.GetPrimAtPath(cube_path).IsValid():
        raise RuntimeError(f"Prop not found on stage: {cube_path}")
    if not collision_rows:
        raise RuntimeError(f"No collision prims found under {_component_root(args.component)}")

    print(f"Recording contact part for {args.component} against {args.prop}")
    print(f"Writing CSV to {args.csv}")
    print(f"Duration={args.duration}s Rate={args.rate_hz}Hz")

    fieldnames = [
        "tool_prim",
        "tool_x",
        "tool_y",
        "tool_z",
        "cube_name",
        "cube_x",
        "cube_y",
        "cube_z",
        "cube_moved",
        "contact_part_prim",
        "contact_part_approximation",
        "contact_part_aabb_distance",
        "contact_part_overlaps_cube",
        "tool_inside_cube_aabb",
        "likely_passthrough",
    ]
    handle, writer = _write_header(args.csv, fieldnames)

    try:
        import omni.usd

        _ = omni.usd.get_context().get_stage()
        sample_dt = 1.0 / max(float(args.rate_hz), 1.0)
        steps = max(1, int(float(args.duration) / sample_dt))
        previous_cube_center = np.asarray(_prim_translation(_prim_world_transform(stage, cube_path)), dtype=float)
        passthrough_state = False
        events_written = 0

        for _ in range(steps):
            loop_start = time.monotonic()

            tool_position = np.asarray(_prim_translation(_prim_world_transform(stage, tool_path)), dtype=float)
            cube_transform = _prim_world_transform(stage, cube_path)
            cube_center = np.asarray(_prim_translation(cube_transform), dtype=float)
            cube_bounds = _prim_bounds(stage, cube_path)
            cube_motion = float(np.linalg.norm(cube_center - previous_cube_center))
            previous_cube_center = cube_center

            part = _pick_contact_part(stage, cube_bounds, collision_rows)
            tool_inside_cube = _inside_aabb(tool_position, cube_bounds["range_min"], cube_bounds["range_max"])

            likely_passthrough = False
            if tool_inside_cube and not passthrough_state:
                passthrough_state = True
            elif not tool_inside_cube and passthrough_state:
                passthrough_state = False
                likely_passthrough = True

            is_touch_event = bool(part["overlap"]) or bool(part["distance"] <= float(args.touch_distance))
            cube_moved = cube_motion >= float(args.prop_move_threshold)
            if is_touch_event or cube_moved or likely_passthrough:
                writer.writerow(
                    {
                        "tool_prim": tool_path,
                        "tool_x": float(tool_position[0]),
                        "tool_y": float(tool_position[1]),
                        "tool_z": float(tool_position[2]),
                        "cube_name": args.prop,
                        "cube_x": float(cube_center[0]),
                        "cube_y": float(cube_center[1]),
                        "cube_z": float(cube_center[2]),
                        "cube_moved": bool(cube_moved),
                        "contact_part_prim": part["prim"],
                        "contact_part_approximation": part["approximation"],
                        "contact_part_aabb_distance": float(part["distance"]),
                        "contact_part_overlaps_cube": bool(part["overlap"]),
                        "tool_inside_cube_aabb": bool(tool_inside_cube),
                        "likely_passthrough": bool(likely_passthrough),
                    }
                )
                events_written += 1

            elapsed = time.monotonic() - loop_start
            if elapsed < sample_dt:
                time.sleep(sample_dt - elapsed)

        handle.flush()
        print(f"Recorded {events_written} interaction rows")
    except KeyboardInterrupt:
        print("Recording stopped by user")
    finally:
        _close_writer(handle)
        print(f"Saved contact trace to {args.csv}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"live_contact_recorder failed: {exc}", file=sys.stderr)
        raise
