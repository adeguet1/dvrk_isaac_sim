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


def _arguments() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="run without an Isaac Sim window")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after this simulation time; zero means run until interrupted")
    parser.add_argument("--update-rate", type=float, default=120.0)
    parser.add_argument("--psm-config", type=Path, default=root / "config/PSM1.yaml")
    parser.add_argument("--ecm-config", type=Path, default=root / "config/ECM.yaml")
    parser.add_argument("--no-psm", action="store_true")
    parser.add_argument("--no-ecm", action="store_true")
    parser.add_argument("--psm-usd", type=Path, help="optional generated PSM USD asset to reference")
    parser.add_argument("--ecm-usd", type=Path, help="optional generated ECM USD asset to reference")
    parser.add_argument("--psm-kinematics", type=Path, help="generated PSM URDF kinematics manifest")
    parser.add_argument("--ecm-kinematics", type=Path, help="generated ECM URDF kinematics manifest")
    return parser.parse_args()


def _reference_usd(path: Path | None, prim_path: str) -> None:
    if path is None:
        return
    if not path.is_file():
        raise FileNotFoundError(f"USD asset not found: {path}")
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(path.resolve()))
    print(f"Referenced USD asset {path} at {prim_path}", flush=True)


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


def main() -> int:
    args = _arguments()

    # Isaac Sim must be initialized before importing most Isaac modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})
    nodes = []
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        simulation_app.update()
        _reference_usd(args.psm_usd, "/World/PSM1")
        if not args.no_ecm:
            _reference_usd(args.ecm_usd, "/World/ECM")

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

        def add_component(namespace: str, config_path: Path):
            manifest = args.psm_kinematics if namespace.startswith("PSM") else args.ecm_kinematics
            config = load_robot_config(config_path, manifest)
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
            nodes.append((node, component, visual))

        if not args.no_psm:
            add_component("PSM1", args.psm_config)
        if not args.no_ecm:
            add_component("ECM", args.ecm_config)

        timeline = get_timeline_interface()
        timeline.play()
        previous_time = float(timeline.get_current_time())
        print("Isaac Sim CRTK smoke test running", flush=True)
        if not args.no_psm:
            print("  PSM1 topics: /PSM1/measured_js, /PSM1/measured_cp, /PSM1/servo_jp", flush=True)
        if not args.no_ecm:
            print("  ECM topics:  /ECM/measured_js, /ECM/measured_cp, /ECM/servo_jp", flush=True)

        while simulation_app.is_running():
            simulation_app.update()
            current_time = float(timeline.get_current_time())
            dt = max(0.0, current_time - previous_time)
            previous_time = current_time

            for node, component, visual in nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
                component.model.step(dt)
                measured = component.model.measured_js()
                if visual is not None:
                    visual.update(measured.names, measured.position)
                component.publish(_ros_time(current_time))

            if args.duration > 0.0 and current_time >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if "rclpy" in locals() and rclpy.ok():
            for node, _, _ in nodes:
                node.destroy_node()
            rclpy.shutdown()
        simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
