"""Start the configured dVRK Isaac Sim scene through Isaac Sim Python."""

from pathlib import Path
import shlex

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from dvrk_isaac_sim.config import load_robot_document


def _resolve(config_path: Path, value, default=None):
    value = default if value in (None, "") else value
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    config_path = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"Simulator configuration not found: {config_path}")
    configured = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(configured, dict):
        raise RuntimeError(f"{config_path}: expected a YAML mapping")

    def value(name, default=None):
        return configured.get(name, default)

    isaac_dir_arg = LaunchConfiguration("isaac_sim_dir").perform(context)
    isaac_dir = Path(isaac_dir_arg).expanduser() if isaac_dir_arg else _resolve(config_path, value("isaac_sim_dir"))
    if isaac_dir is None:
        raise RuntimeError("Isaac Sim path is not configured. Set isaac_sim_dir:=... or isaac_sim_dir in the config file.")
    isaac_dir = isaac_dir.resolve()
    isaac_python = isaac_dir / "python.sh"
    if not isaac_python.is_file():
        raise RuntimeError(f"Isaac Sim python.sh not found: {isaac_python}")

    scene_value = LaunchConfiguration("scene").perform(context) or value("scene")
    scenes_dir = config_path.parent / "scenes"
    if scene_value in (None, ""):
        available = sorted(scenes_dir.glob("*.yaml"))
        names = "\n  ".join(path.name for path in available) or "(none)"
        raise RuntimeError(f"No scene selected. Set scene:=... or scene in {config_path}.\nAvailable scenes:\n  {names}")
    scene_config = Path(str(scene_value)).expanduser()
    if not scene_config.is_absolute():
        candidate = (config_path.parent / scene_config).resolve()
        if not candidate.is_file() and scene_config.parent == Path("."):
            candidate = (scenes_dir / scene_config).resolve()
        if not candidate.suffix:
            candidate = candidate.with_suffix(".yaml")
        scene_config = candidate
    scene_config = scene_config.resolve()
    generated_dir = _resolve(config_path, value("generated_dir"))
    if generated_dir is None:
        raise RuntimeError(f"{config_path}: generated_dir is required")

    conversion_commands = []
    converter = package_share / "scripts" / "convert_dvrk_model.py"

    def ensure_asset(model, robot_type, variant):
        asset_name = f"{model}_{variant}"
        asset_path = generated_dir / asset_name / model / f"{model}.usda"
        manifest_path = generated_dir / asset_name / "kinematics.json"
        if asset_path.is_file() and manifest_path.is_file():
            return
        command = [str(isaac_python), str(converter), "--model", model,
                   "--output-dir", str(generated_dir), "--asset-name", asset_name, "--force"]
        command.extend(["--instrument", variant] if robot_type == "PSM" else ["--endoscope", variant])
        conversion_commands.append(command)

    if not scene_config.is_file():
        raise RuntimeError(f"Scene configuration not found: {scene_config}")
    scene_document = yaml.safe_load(scene_config.read_text(encoding="utf-8")) or {}
    scene = scene_document.get("scene", scene_document)
    for configured_robot in scene.get("robots", []):
        options = configured_robot if isinstance(configured_robot, dict) else {}
        robot_config = Path(options.get("config", configured_robot))
        if not robot_config.is_absolute():
            robot_config = (scene_config.parent.parent.parent / robot_config).resolve()
        robot = load_robot_document(robot_config).get("robot", {})
        robot_type = str(robot.get("type", "")).upper()
        model = str(robot.get("name"))
        variant = (options.get("instrument", "420006") if robot_type == "PSM"
                   else options.get("endoscope", "Si_straight"))
        ensure_asset(model, robot_type, str(variant))

    command = [str(isaac_python), str(package_share / "scripts" / "run_sim.py"),
               "--config", str(config_path), "--scene", str(scene_config)]
    headless = LaunchConfiguration("headless").perform(context).lower()
    if headless in {"true", "1", "yes"}:
        command.append("--headless")
    duration = LaunchConfiguration("duration").perform(context)
    if duration and float(duration) > 0.0:
        command.extend(["--duration", duration])

    ros_distro = LaunchConfiguration("ros_distro").perform(context) or str(value("ros_distro", "jazzy"))
    rmw_implementation = LaunchConfiguration("rmw_implementation").perform(context) or str(value("rmw_implementation", "rmw_fastrtps_cpp"))
    environment = {
        "ROS_DISTRO": ros_distro,
        "RMW_IMPLEMENTATION": rmw_implementation,
        "PYTHONUNBUFFERED": "1",
    }
    if conversion_commands:
        shell_command = " && ".join([shlex.join(item) for item in conversion_commands] + [shlex.join(command)])
        return [LogInfo(msg=f"Generating {len(conversion_commands)} missing USD/URDF asset set(s) from dvrk_model"),
                ExecuteProcess(cmd=["bash", "-c", shell_command], cwd=str(isaac_dir),
                               additional_env=environment, output="screen")]
    return [LogInfo(msg=f"Starting Isaac Sim scene {scene_config.stem}"),
            ExecuteProcess(cmd=command, cwd=str(isaac_dir), additional_env=environment, output="screen")]


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    default_config = str(package_share / "config" / "isaac_sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config,
                              description="Saved simulator config YAML"),
        DeclareLaunchArgument("isaac_sim_dir", default_value="",
                              description="Optional Isaac Sim path override"),
        DeclareLaunchArgument("scene", default_value="",
                              description="Scene YAML path or filename under config/scenes"),
        DeclareLaunchArgument("headless", default_value="",
                              description="Optional headless override; otherwise use config"),
        DeclareLaunchArgument("duration", default_value="",
                              description="Optional duration override in seconds"),
        DeclareLaunchArgument("ros_distro", default_value=""),
        DeclareLaunchArgument("rmw_implementation", default_value=""),
        OpaqueFunction(function=_start_sim),
    ])
