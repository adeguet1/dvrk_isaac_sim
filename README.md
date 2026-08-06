# dVRK Isaac Sim

`dvrk_isaac_sim` is a lightweight simulated patient-side robot environment for Isaac Sim 6.0 and ROS 2.

The project focuses on software integration and kinematic simulation rather than hardware or dynamic fidelity. It provides configurable virtual PSMs and an ECM, dVRK/CRTK-style interfaces, selectable dVRK instrument visuals, and a rendered endoscope view.

## Current scope

- ROS 2 only.
- Isaac Sim 6.0.
- Kinematic PSM control with `yaw`, `pitch`, `insertion`, `roll`, `wrist_pitch`, and `wrist_yaw`; jaw control is provided as a separate logical one-joint interface.
- Kinematic ECM control with `yaw`, `pitch`, `insertion`, and `roll`.
- Startup profiles for two or three PSMs plus an ECM.
- Reuse of visual assets from [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model).
- YAML-defined robot base frames, with future TF support planned.
- ROS image transport for the ECM view.

Full patient-cart CAD, dynamics, contact simulation, and hardware-runtime dependencies are outside the project scope.

## Documentation

- [Installation and runtime environment](docs/installation.md)
- [Design specification](docs/design.md)
- [Frames and conventions](docs/frames.md)
- [ROS 2 interface](docs/ros_interface.md)
- [PSM configuration](config/PSM1.yaml)
- [ECM configuration](config/ECM.yaml)
- [Two-PSM scene](config/scenes/ECM_PSM1_PSM2.yaml)
- [Three-PSM scene](config/scenes/ECM_PSM1_PSM2_PSM3.yaml)

## dVRK resources

- [Official dVRK documentation](https://dvrk.readthedocs.io/)
- [dVRK ROS and software documentation](https://dvrk.readthedocs.io/main/)
- [`sawIntuitiveResearchKit`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit)
- [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model)
- [dVRK ROS 2 software documentation](https://dvrk.readthedocs.io/main/pages/software/ros-2.html)

The simulator reuses dVRK model assets and interface conventions but does not require the dVRK runtime, cisst, or SAW.

The recommended workflow is to build and launch through ROS 2:

```bash
mkdir -p /path/to/isaac_sim_ws/src
cd /path/to/isaac_sim_ws/src
git clone https://github.com/collaborative-robotics/crtk_msgs.git
git clone https://github.com/jhu-dvrk/dvrk_model.git
git clone /path/to/dvrk_isaac_sim

cd /path/to/isaac_sim_ws/src/dvrk_isaac_sim
export ISAAC_SIM_DIR=/path/to/isaac-sim
./scripts/build.sh

source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
ros2 launch dvrk_isaac_sim run_sim.launch.py
```

The initial ROS 2 adapter can be run for one configured component:

```bash
ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args -r __ns:=/PSM1 \
  -p robot_config:=/path/to/config/PSM1.yaml
```

## Tested teleoperation

The ROS 2 interface has been tested with the dVRK system configuration using an old Logitech 3Dconnexion SpaceBall 5000 as the MTMR input and the simulated `/PSM1` as the puppet. The corresponding dVRK configuration is `system-MTMR-3Dconnexion-PSM1_from_ROS.json` in `saw3Dconnexion/share`.

Start Isaac Sim first, source the Isaac Sim ROS 2 workspace, and launch the PSM/ECM simulation. Then start the dVRK system with the SpaceBall configuration. The PSM should report `ENABLED` and `is_homed=true`; MTMR motion should teleoperate the simulated PSM Cartesian pose and jaw.
