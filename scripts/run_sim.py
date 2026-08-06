#!/usr/bin/env python3
"""Isaac Sim 6.0 ROS 2/CRTK smoke test.

This script intentionally uses no USD robot assets yet. It validates that:

* Isaac Sim starts with the ROS 2 bridge;
* the sourced ROS 2 Python environment is visible inside Isaac Sim;
* crtk_msgs custom messages can be imported;
* PSM1 and ECM publish CRTK topics using simulation time; and
* joint and Cartesian commands are delivered to the kinematic models.

Run with Isaac Sim's Python interpreter, not the system Python:

    ./python.sh /path/to/dvrk_isaac_sim/scripts/run_sim.py --headless
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml


def _arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run without an Isaac Sim window")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this simulation time; zero means run until interrupted")
    parser.add_argument("--update-rate", type=float, default=120.0)
    parser.add_argument("--psm-config", type=Path, default=root / "config/PSM1.yaml")
    parser.add_argument("--ecm-config", type=Path, default=root / "config/ECM.yaml")
    parser.add_argument("--scene-config", type=Path, default=root / "config/scenes/ECM_PSM1_PSM2_PSM3.yaml")
    parser.add_argument("--generated-dir", type=Path, default=root.parent.parent / ".generated" / "isaacsim-6.0")
    parser.add_argument("--instrument", default="420006")
    parser.add_argument("--endoscope", default="Si_straight")
    parser.add_argument("--camera", choices=("off", "mono", "stereo"), default="mono")
    parser.add_argument("--renderer", choices=("MinimalRendering", "RaytracedLighting", "RealTimePathTracing", "PathTracing"), default="MinimalRendering")
    parser.add_argument("--no-psm", action="store_true")
    parser.add_argument("--no-ecm", action="store_true")
    parser.add_argument("--psm-usd", type=Path, help="optional generated PSM USD asset to reference")
    parser.add_argument("--ecm-usd", type=Path, help="optional generated ECM USD asset to reference")
    parser.add_argument("--psm-kinematics", type=Path, help="generated PSM URDF kinematics manifest")
    parser.add_argument("--ecm-kinematics", type=Path, help="generated ECM URDF kinematics manifest")
    return parser.parse_args()


def _reference_usd(path: Path | None, prim_path: str, position=None, orientation_xyzw=None) -> None:
    if path is None:
        return
    if not path.is_file():
        raise FileNotFoundError(f"USD asset not found: {path}")
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(path.resolve()))
    if position is not None or orientation_xyzw is not None:
        from pxr import Gf, UsdGeom
        xform = UsdGeom.Xformable(prim)
        if position is not None:
            xform.AddTranslateOp(opSuffix="base").Set(Gf.Vec3d(*position))
        if orientation_xyzw is not None:
            x, y, z, w = orientation_xyzw
            # AddOrientOp currently causes a native USD shutdown in Isaac Sim
            # 6.0 when appended to imported robot xform stacks.  Author the
            # equivalent XYZ Euler op in degrees instead.
            roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            xform.AddRotateXYZOp(opSuffix="base").Set(
                Gf.Vec3d(*np.degrees([roll, pitch, yaw])))
    print(f"Referenced USD asset {path} at {prim_path}", flush=True)


def _setup_scene_lighting() -> None:
    """Add neutral local lighting so dark instruments remain visible."""
    import omni.usd
    from pxr import Gf, UsdLux, UsdGeom

    stage = omni.usd.get_context().get_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/DomeLight")
    dome.CreateColorAttr(Gf.Vec3f(0.22, 0.22, 0.22))
    dome.CreateIntensityAttr(450.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Lighting/KeyLight")
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))
    key.CreateIntensityAttr(1800.0)
    key.CreateAngleAttr(0.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3d(35.0, -25.0, -30.0))


def _ros_time(seconds: float):
    from builtin_interfaces.msg import Time

    seconds = max(0.0, float(seconds))
    result = Time()
    result.sec = int(seconds)
    result.nanosec = int(round((seconds - result.sec) * 1e9))
    if result.nanosec >= 1_000_000_000:
        result.sec += 1
        result.nanosec -= 1_000_000_000
    return result


def _scene_entries(scene_path: Path, root: Path) -> list[dict]:
    document = yaml.safe_load(scene_path.read_text(encoding="utf-8")) or {}
    scene = document.get("scene", document)
    frames = scene.get("frames", {})
    entries = []
    for configured in scene.get("robots", []):
        config_path = Path(configured)
        if not config_path.is_absolute():
            config_path = scene_path.parent.parent.parent / config_path
        config_path = config_path.resolve()
        robot_document = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        name = str(robot_document["robot"]["name"])
        frame = frames.get(name, {})
        entries.append({"name": name, "config": config_path, "frame": frame})
    if not entries:
        raise ValueError(f"scene has no robots: {scene_path}")
    return entries

def _generated_variant(args: argparse.Namespace, entry: dict) -> tuple[Path, Path]:
    robot = yaml.safe_load(entry["config"].read_text(encoding="utf-8"))["robot"]
    name = entry["name"]
    variant = args.instrument if robot["type"] == "psm" else args.endoscope
    asset_dir = args.generated_dir.expanduser().resolve() / f"{name}_{variant}"
    return asset_dir / name / f"{name}.usda", asset_dir / "kinematics.json"

def main() -> int:
    args = _arguments()

    # Isaac Sim must be initialized before importing most Isaac modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless, "renderer": args.renderer})
    nodes = []
    ui_window = None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        if args.scene_config and args.scene_config.is_file() and not args.no_psm and not args.no_ecm:
            scene_entries = _scene_entries(args.scene_config, Path(__file__).resolve().parents[1])
        else:
            scene_entries = []
        if scene_entries:
            for entry in scene_entries:
                asset, _ = _generated_variant(args, entry)
                robot_type = yaml.safe_load(entry["config"].read_text(encoding="utf-8"))["robot"]["type"]
                if robot_type == "psm":
                    frame = entry.get("frame", {})
                    _reference_usd(asset, f"/World/{entry['name']}",
                                   frame.get("position"), frame.get("orientation_xyzw"))
        else:
            _reference_usd(args.psm_usd, "/World/PSM1")
            # The ECM is intentionally represented by kinematics and its
            # camera only; never add the endoscope/ECM mesh to the stage.

        _setup_scene_lighting()

        import rclpy
        from rclpy.node import Node
        from crtk_msgs.msg import OperatingState
        from omni.timeline import get_timeline_interface

        # This import is intentional: it is the custom-message preflight check.
        print(f"Loaded custom ROS 2 message: {OperatingState.__module__}.OperatingState", flush=True)

        package_root = Path(__file__).resolve().parents[1]
        if str(package_root) not in sys.path:
            sys.path.insert(0, str(package_root))

        from dvrk_isaac_sim.config import load_robot_config
        from dvrk_isaac_sim.kinematics import CrtkECM, CrtkPSM
        from dvrk_isaac_sim.ros_interface import CrtkRosComponent
        from dvrk_isaac_sim.usd_visual import CrtkUsdVisual

        rclpy.init()

        cameras = []

        def add_component(namespace: str, config_path: Path, frame: dict | None = None, manifest: Path | None = None):
            frame = frame or {}
            config = load_robot_config(
                config_path, manifest,
                frame.get("position"), frame.get("orientation_xyzw")
            )
            if config.type == "psm":
                model = CrtkPSM(config)
            elif config.type == "ecm":
                model = CrtkECM(config)
            else:
                raise ValueError(f"Unsupported robot type: {config.type}")
            node = Node(f"dvrk_isaac_sim_{config.name}", namespace=f"/{namespace}")
            component = CrtkRosComponent(node, config, model)
            import omni.usd

            stage = omni.usd.get_context().get_stage()
            visual = (
                CrtkUsdVisual(config.name)
                if stage.GetPrimAtPath(f"/World/{config.name}").IsValid()
                else None
            )
            camera = None
            if config.type == "ecm" and args.camera != "off":
                from dvrk_isaac_sim.camera import IsaacCameraPublisher
                camera = IsaacCameraPublisher(node, config, args.camera)
                cameras.append(camera)
            nodes.append((node, component, visual, camera))

        if scene_entries:
            for entry in scene_entries:
                asset, manifest = _generated_variant(args, entry)
                add_component(entry["name"], entry["config"], entry["frame"], manifest)
        else:
            if not args.no_psm:
                add_component("PSM1", args.psm_config, manifest=args.psm_kinematics)
            if not args.no_ecm:
                add_component("ECM", args.ecm_config, manifest=args.ecm_kinematics)

        if not args.headless:
            from dvrk_isaac_sim.isaac_ui import IsaacCrtkWindow
            ui_window = IsaacCrtkWindow([component for _, component, _, _ in nodes])

        timeline = get_timeline_interface()
        timeline.play()
        previous_time = float(timeline.get_current_time())
        print("Isaac Sim CRTK smoke test running", flush=True)
        for entry in scene_entries or ([{"name": "PSM1"}] if not args.no_psm else []):
            print(f"  {entry['name']} topics: /{entry['name']}/measured_js, /{entry['name']}/measured_cp, /{entry['name']}/servo_jp", flush=True)
        if not scene_entries and not args.no_ecm:
            print("  ECM topics: /ECM/measured_js, /ECM/measured_cp, /ECM/servo_jp", flush=True)

        while simulation_app.is_running():
            simulation_app.update()
            current_time = float(timeline.get_current_time())
            dt = max(0.0, current_time - previous_time)
            previous_time = current_time

            for node, component, visual, camera in nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
                component.model.step(dt)
                measured = component.model.measured_js()
                if visual is not None:
                    visual.update(measured.names, measured.position)
                component.publish(_ros_time(current_time))
                if camera is not None:
                    camera.publish(current_time, component.model.measured_cp())
            if ui_window is not None:
                ui_window.update()

            if args.duration > 0.0 and current_time >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if ui_window is not None:
            ui_window.close()
        if "rclpy" in locals() and rclpy.ok():
            for node, _, _, _ in nodes:
                node.destroy_node()
            rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
