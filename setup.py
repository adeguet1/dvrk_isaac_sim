from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_isaac_sim"
local_isaac_config = Path("share/isaac_sim.yaml")
config_files = [
    "share/arms/PSM.yaml",
    "share/arms/PSM1.yaml",
    "share/arms/PSM2.yaml",
    "share/arms/PSM3.yaml",
    "share/arms/ECM.yaml",
]
data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/share/arms", [
        *config_files,
    ]),
    (f"share/{package_name}/share/scenes", [
        str(path) for path in sorted(Path("share/scenes").glob("*.yaml"))
    ]),
    (f"share/{package_name}/share/dvrk_systems", [
        str(path) for path in sorted(Path("share/dvrk_systems").glob("*.json"))
    ]),
]
if local_isaac_config.exists():
    data_files.append((f"share/{package_name}/share", [str(local_isaac_config)]))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        *data_files,
        (f"share/{package_name}/launch", ["launch/simulator.launch.py"]),
        (f"share/{package_name}/scripts", [
            "scripts/simulator.py",
            "scripts/tests",
            "scripts/convert_dvrk_model.py",
            "scripts/generate_cart_frames.py",
            "scripts/validate_config.py",
        ]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dvrk_isaac_sim_kinematics = dvrk_isaac_sim.kinematics:main",
            "dvrk_isaac_sim_ros = dvrk_isaac_sim.ros_node:main",
        ],
    },
)
