from setuptools import find_packages, setup


package_name = "dvrk_isaac_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", [
            "config/PSM1.yaml",
            "config/PSM2.yaml",
            "config/PSM3.yaml",
            "config/ECM.yaml",
        ]),
        (f"share/{package_name}/config/scenes", [
            "config/scenes/ECM_PSM1_PSM2.yaml",
            "config/scenes/ECM_PSM1_PSM2_PSM3.yaml",
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
