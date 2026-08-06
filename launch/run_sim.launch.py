"""Start the dVRK Isaac Sim smoke test through the Isaac Sim Python launcher."""

from pathlib import Path
import os
import shlex

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _start_sim(context):
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    configured_path = package_share / "config" / "isaac_sim.yaml"
    configured = {}
    if configured_path.exists():
        with configured_path.open("r", encoding="utf-8") as stream:
            configured = yaml.safe_load(stream) or {}

    isaac_dir = LaunchConfiguration("isaac_sim_dir").perform(context)
    if not isaac_dir:
        isaac_dir = configured.get("isaac_sim_dir", "")
    if not isaac_dir:
        raise RuntimeError(
            "Isaac Sim path is not configured. Set ISAAC_SIM_DIR and run scripts/build.sh, "
            "or pass isaac_sim_dir:=/path/to/isaac-sim."
        )

    arm = LaunchConfiguration("arm").perform(context).upper()
    valid_arms = {"PSM1", "PSM2", "PSM3", "ECM"}
    if arm and arm not in valid_arms:
        raise RuntimeError(f"Unsupported arm '{arm}'. Choose one of: {', '.join(sorted(valid_arms))}")

    generated_dir = LaunchConfiguration("generated_dir").perform(context)
    generated_dir = Path(generated_dir).expanduser().resolve()

    asset = ""
    manifest = ""
    conversion_command = None
    if arm:
        variant = (
            LaunchConfiguration("instrument").perform(context)
            if arm.startswith("PSM")
            else LaunchConfiguration("endoscope").perform(context)
        )
        asset_name = f"{arm}_{variant}"
        asset_path = generated_dir / asset_name / arm / f"{arm}.usda"
        manifest_path = generated_dir / asset_name / "kinematics.json"
        asset = str(asset_path)
        manifest = str(manifest_path)
        if not asset_path.is_file() or not manifest_path.is_file():
            converter = package_share / "scripts" / "convert_dvrk_model.py"
            converter_command = [
                str(Path(isaac_dir) / "python.sh"),
                str(converter),
                "--model", arm,
                "--output-dir", str(generated_dir),
                "--asset-name", asset_name,
                "--force",
            ]
            if arm.startswith("PSM"):
                converter_command.extend(["--instrument", LaunchConfiguration("instrument").perform(context)])
            else:
                converter_command.extend(["--endoscope", LaunchConfiguration("endoscope").perform(context)])
            conversion_command = converter_command

    isaac_python = Path(isaac_dir) / "python.sh"
    script = package_share / "scripts" / "run_sim.py"
    command = [str(isaac_python), str(script)]
    if LaunchConfiguration("headless").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--headless")
    duration = LaunchConfiguration("duration").perform(context)
    if duration and float(duration) > 0.0:
        command.extend(["--duration", duration])
    if arm == "ECM":
        command.append("--no-psm")
        command.extend(["--ecm-usd", asset, "--ecm-kinematics", manifest])
        command.extend(["--ecm-config", str(package_share / "config" / "ECM.yaml")])
    elif arm.startswith("PSM"):
        command.append("--no-ecm")
        command.extend(["--psm-usd", asset, "--psm-kinematics", manifest])
        command.extend(["--psm-config", str(package_share / "config" / f"{arm}.yaml")])

    for argument, launch_argument in (("psm_usd", "psm_usd"), ("ecm_usd", "ecm_usd")):
        configured_asset = LaunchConfiguration(launch_argument).perform(context)
        if configured_asset and not arm:
            command.extend([f"--{argument.replace('_', '-')}", configured_asset])

    environment = {
        "ROS_DISTRO": LaunchConfiguration("ros_distro").perform(context),
        "RMW_IMPLEMENTATION": LaunchConfiguration("rmw_implementation").perform(context),
        "PYTHONUNBUFFERED": "1",
    }
    if conversion_command is not None:
        # Keep conversion and runtime startup in one sequential action. This
        # avoids launch event-handler races while Isaac Sim is shutting down.
        shell_command = (
            shlex.join(conversion_command)
            + " && test -f "
            + shlex.quote(asset)
            + " && "
            + shlex.join(command)
        )
        return [
            LogInfo(msg=f"USD asset missing for {arm}; converting to {asset}"),
            ExecuteProcess(cmd=["bash", "-c", shell_command], cwd=str(isaac_dir), additional_env=environment, output="screen"),
        ]
    return [
        LogInfo(msg=f"Starting Isaac Sim with {arm or 'configured ROS components'}"),
        ExecuteProcess(cmd=command, cwd=str(isaac_dir), additional_env=environment, output="screen"),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("dvrk_isaac_sim"))
    default_generated_dir = os.environ.get(
        "DVRK_ISAAC_SIM_GENERATED_DIR",
        str(package_share.parents[3] / ".generated" / "isaacsim-6.0"),
    )
    return LaunchDescription([
        DeclareLaunchArgument("isaac_sim_dir", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("duration", default_value="0.0"),
        DeclareLaunchArgument("arm", default_value=""),
        DeclareLaunchArgument("generated_dir", default_value=default_generated_dir),
        DeclareLaunchArgument("instrument", default_value="420006"),
        DeclareLaunchArgument("endoscope", default_value="Si_straight"),
        DeclareLaunchArgument("psm_usd", default_value=""),
        DeclareLaunchArgument("ecm_usd", default_value=""),
        DeclareLaunchArgument("ros_distro", default_value="jazzy"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
        OpaqueFunction(function=_start_sim),
    ])
