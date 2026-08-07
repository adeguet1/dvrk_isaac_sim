from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_isaac_sim"
local_isaac_config = Path("config/isaac_sim.yaml")
config_files = [
    "config/PSM.yaml",
    "config/PSM1.yaml",
    "config/PSM2.yaml",
    "config/PSM3.yaml",
    "config/ECM.yaml",
]
if local_isaac_config.exists():
    config_files.append(str(local_isaac_config))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", [
            *config_files,
        ]),
        (f"share/{package_name}/config/scenes", [
            str(path) for path in sorted(Path("config/scenes").glob("*.yaml"))
        ]),
        (f"share/{package_name}/launch", ["launch/run_sim.launch.py"]),
        (f"share/{package_name}/scripts", [
            "scripts/run_sim.py",
            "scripts/convert_dvrk_model.py",
            "scripts/generate_cart_frames.py",
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dvrk_isaac_sim_kinematics = dvrk_isaac_sim.kinematics:main",
            "dvrk_isaac_sim_ros = dvrk_isaac_sim.ros_interface:main",
        ],
    },
)
