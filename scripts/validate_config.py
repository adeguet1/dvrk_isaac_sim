#!/usr/bin/env python3
"""Validate simulator config and all scene YAML files without Isaac Sim."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# Permit running the source script directly.
_package_root = Path(__file__).resolve().parents[1]
if str(_package_root) not in sys.path:
    sys.path.insert(0, str(_package_root))

from dvrk_isaac_sim.scene import (
    available_environment_paths,
    available_scene_paths,
    load_scene,
    load_simulator_config,
)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "share" / "isaac_sim.yaml")
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    simulator = load_simulator_config(config_path)
    scenes = available_scene_paths(config_path)
    environments = available_environment_paths(config_path)
    for scene_path in scenes:
        scene = load_scene(scene_path)
        print(
            f"OK {scene_path.name}: {len(scene.robots)} robots, "
            f"{len(scene.props)} props, camera={scene.camera.mode}"
        )
    for environment_path in environments:
        environment = load_scene(environment_path)
        print(
            f"OK {environment_path.name}: {len(environment.robots)} robots, "
            f"{len(environment.props)} props, camera={environment.camera.mode}"
        )
    print(
        f"OK {config_path.name}: renderer={simulator.renderer}, "
        f"scenes={len(scenes)}, environments={len(environments)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
