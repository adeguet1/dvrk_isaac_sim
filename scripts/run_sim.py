#!/usr/bin/env python3
"""Isaac Sim 6.0 ROS 2/CRTK smoke test.

This script intentionally uses no USD robot assets yet. It validates that:

* Isaac Sim starts with the ROS 2 bridge;
* the sourced ROS 2 Python environment is visible inside Isaac Sim;
* crtk_msgs custom messages can be imported;
* PSM1 and ECM publish CRTK topics using simulation time; and
* joint and Cartesian commands are delivered to the kinematic models.

Run with Isaac Sim's Python interpreter, not the system Python. Runtime settings are
loaded from config; use --config to select a saved YAML file.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

# Permit running the source script directly with Isaac Sim's Python.
_package_root = Path(__file__).resolve().parents[1]
if str(_package_root) not in sys.path:
    sys.path.insert(0, str(_package_root))
from dvrk_isaac_sim.config import load_robot_document


def _arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config/isaac_sim.yaml",
                        help="YAML file containing simulator settings")
    parser.add_argument("--headless", action="store_true", default=None,
                        help="override config and run without an Isaac Sim window")
    parser.add_argument("--duration", type=float, default=None,
                        help="override config simulation duration in seconds")
    parser.add_argument("--scene", type=Path, default=None,
                        help="override config and select a scene YAML")
    args = parser.parse_args()

    config_path = args.config.expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        settings = yaml.safe_load(stream) or {}
    if not isinstance(settings, dict):
        raise ValueError(f"{config_path}: expected a YAML mapping")

    def setting(name: str, default=None):
        return settings.get(name, default)

    def path_setting(name: str, default=None):
        value = setting(name, default)
        if value in (None, ""):
            return None
        value = Path(value).expanduser()
        return value if value.is_absolute() else (config_path.parent / value).resolve()

    args.config = config_path
    scene_value = args.scene if args.scene is not None else setting("scene")
    scenes_dir = config_path.parent / "scenes"
    if scene_value in (None, ""):
        available = sorted(scenes_dir.glob("*.yaml"))
        names = "\n  ".join(path.name for path in available) or "(none)"
        raise ValueError(f"No scene selected. Choose --scene or set scene in {config_path}.\nAvailable scenes:\n  {names}")
    scene_path = Path(str(scene_value)).expanduser()
    if not scene_path.is_absolute():
        candidate = (config_path.parent / scene_path).resolve()
        if not candidate.is_file() and scene_path.parent == Path("."):
            candidate = (scenes_dir / scene_path).resolve()
        if not candidate.suffix:
            candidate = candidate.with_suffix(".yaml")
        scene_path = candidate
    args.scene_config = scene_path.resolve()
    args.generated_dir = path_setting("generated_dir", root.parent.parent / ".generated" / "isaacsim-6.0")
    args.renderer = str(setting("renderer", "RaytracedLighting"))
    args.headless = bool(setting("headless", False)) if args.headless is None else args.headless
    args.duration = float(setting("duration", 0.0)) if args.duration is None else args.duration
    scene_document = yaml.safe_load(args.scene_config.read_text(encoding="utf-8")) or {}
    scene_settings = scene_document.get("scene", scene_document)
    args.scene_camera = scene_settings.get("camera", {}) or {}
    args.camera = str(args.scene_camera.get("mode", "mono"))
    if args.camera not in {"off", "mono", "stereo"}:
        raise ValueError(f"{args.scene_config}: camera.mode must be off, mono, or stereo")
    if args.renderer not in {"RaytracedLighting", "RealTimePathTracing", "PathTracing"}:
        raise ValueError(f"{config_path}: unsupported renderer {args.renderer}")
    return args


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
        options = configured if isinstance(configured, dict) else {}
        configured_path = options.get("config", configured) if isinstance(configured, dict) else configured
        config_path = Path(configured_path)
        if not config_path.is_absolute():
            config_path = scene_path.parent.parent.parent / config_path
        config_path = config_path.resolve()
        robot_document = load_robot_document(config_path)
        name = str(robot_document["robot"]["name"])
        frame = frames.get(name, {})
        entries.append({"name": name, "config": config_path, "frame": frame,
                        "instrument": options.get("instrument"),
                        "endoscope": options.get("endoscope")})
    if not entries:
        raise ValueError(f"scene has no robots: {scene_path}")
    return entries

def _generated_variant(args: argparse.Namespace, entry: dict) -> tuple[Path, Path]:
    robot = load_robot_document(entry["config"])["robot"]
    name = entry["name"]
    variant = (entry.get("instrument") or "420006"
               if str(robot["type"]).upper() == "PSM"
               else entry.get("endoscope") or "Si_straight")
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
        if not args.scene_config.is_file():
            raise FileNotFoundError(f"Scene configuration not found: {args.scene_config}")
        scene_entries = _scene_entries(args.scene_config, Path(__file__).resolve().parents[1])
        if scene_entries:
            for entry in scene_entries:
                asset, _ = _generated_variant(args, entry)
                robot_type = load_robot_document(entry["config"])["robot"]["type"].upper()
                if robot_type == "PSM":
                    frame = entry.get("frame", {})
                    _reference_usd(asset, f"/World/{entry['name']}",
                                   frame.get("position"), frame.get("orientation_xyzw"))
        # The ECM is intentionally represented by kinematics and its camera
        # only; never add the endoscope/ECM mesh to the stage.

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
            if config.type == "PSM":
                model = CrtkPSM(config)
            elif config.type == "ECM":
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
            if config.type == "ECM" and args.camera != "off":
                from dvrk_isaac_sim.camera import IsaacCameraPublisher
                camera = IsaacCameraPublisher(node, config, args.camera, args.scene_camera)
                cameras.append(camera)
            nodes.append((node, component, visual, camera))

        if scene_entries:
            for entry in scene_entries:
                asset, manifest = _generated_variant(args, entry)
                add_component(entry["name"], entry["config"], entry["frame"], manifest)

        # PSM Cartesian CRTK topics are expressed in the moving ECM optical
        # frame, as on a dVRK system. Wire this after all components exist
        # because scene YAML may list the PSMs before the ECM.
        ecm_component = next((component for _, component, _, _ in nodes
                              if component.config.type == "ECM"), None)
        if ecm_component is not None:
            # PSM Cartesian topics use dVRK view axes, derived from the
            # current ECM optical FK rather than raw optical-camera axes.
            ecm_view_frame = f"{ecm_component.config.name}_view"
            for _, component, _, _ in nodes:
                if component.config.type == "PSM":
                    component.set_cartesian_reference(ecm_component.model, ecm_view_frame)

        if not args.headless:
            from dvrk_isaac_sim.isaac_ui import IsaacCrtkWindow
            ui_window = IsaacCrtkWindow([component for _, component, _, _ in nodes])

        # Advance ECM first so every PSM reads the current, not previous-step,
        # camera pose when converting Cartesian state and commands.
        ordered_nodes = sorted(
            nodes, key=lambda item: 0 if item[1].config.type == "ECM" else 1
        )

        timeline = get_timeline_interface()
        timeline.play()
        previous_time = float(timeline.get_current_time())
        print("Isaac Sim CRTK smoke test running", flush=True)
        for entry in scene_entries:
            print(f"  {entry['name']} topics: /{entry['name']}/measured_js, /{entry['name']}/measured_cp, /{entry['name']}/servo_jp", flush=True)

        while simulation_app.is_running():
            simulation_app.update()
            current_time = float(timeline.get_current_time())
            dt = max(0.0, current_time - previous_time)
            previous_time = current_time

            for node, component, visual, camera in ordered_nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
                component.model.step(dt)
                measured = component.model.measured_js()
                if visual is not None:
                    visual.update(
                        measured.names, measured.position,
                        component.jaw_position,
                    )
                component.publish(_ros_time(current_time))
                if camera is not None:
                    camera.publish(current_time, component.model.measured_cp())
            if ui_window is not None:
                ui_window.update()

            if args.duration > 0.0 and current_time >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    except Exception:
        # Isaac Sim can close its window immediately after a Python exception.
        # Print the full traceback before cleanup so asset/conversion failures
        # remain actionable from the launch console.
        import traceback
        traceback.print_exc()
        raise
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
