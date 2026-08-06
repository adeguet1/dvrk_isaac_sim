"""Start the dVRK Isaac Sim smoke test through the Isaac Sim Python launcher."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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

    isaac_python = Path(isaac_dir) / "python.sh"
    script = package_share / "scripts" / "run_sim.py"
    command = [str(isaac_python), str(script)]
    if LaunchConfiguration("headless").perform(context).lower() in {"true", "1", "yes"}:
        command.append("--headless")
    duration = LaunchConfiguration("duration").perform(context)
    if duration and float(duration) > 0.0:
        command.extend(["--duration", duration])

    return [
        ExecuteProcess(
            cmd=command,
            cwd=str(isaac_dir),
            additional_env={
                "ROS_DISTRO": LaunchConfiguration("ros_distro").perform(context),
                "RMW_IMPLEMENTATION": LaunchConfiguration("rmw_implementation").perform(context),
            },
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("isaac_sim_dir", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("duration", default_value="0.0"),
        DeclareLaunchArgument("ros_distro", default_value="jazzy"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_fastrtps_cpp"),
        OpaqueFunction(function=_start_sim),
    ])
