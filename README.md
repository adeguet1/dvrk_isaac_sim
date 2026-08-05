# dVRK Isaac Sim

`dvrk_isaac_sim` is a lightweight simulated patient-side robot environment for Isaac Sim 6.0 and ROS 2.

The project focuses on software integration and kinematic simulation rather than hardware or dynamic fidelity. It provides configurable virtual PSMs and an ECM, dVRK/CRTK-style interfaces, selectable dVRK instrument visuals, and a rendered endoscope view.

## Current scope

- ROS 2 only.
- Isaac Sim 6.0.
- Kinematic PSM control with `yaw`, `pitch`, and `insertion`.
- Kinematic ECM control with `yaw`, `pitch`, `insertion`, and `roll`.
- Startup profiles for two or three PSMs plus an ECM.
- Reuse of visual assets from [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model).
- YAML-defined robot base frames, with future TF support planned.
- ROS image transport for the ECM view.

Full patient-cart CAD, dynamics, contact simulation, and hardware-runtime dependencies are outside the project scope.

## Documentation

- [Design specification](docs/design.md)
- [Frames and conventions](docs/frames.md)
- [ROS 2 interface](docs/ros_interface.md)
- [PSM configuration](config/psm1.yaml)
- [ECM configuration](config/ecm.yaml)
- [Two-PSM scene](config/scenes/psm2_ecm.yaml)
- [Three-PSM scene](config/scenes/psm3_ecm.yaml)

## dVRK resources

- [Official dVRK documentation](https://dvrk.readthedocs.io/)
- [dVRK ROS and software documentation](https://dvrk.readthedocs.io/main/)
- [`sawIntuitiveResearchKit`](https://github.com/jhu-dvrk/sawIntuitiveResearchKit)
- [`dvrk_model`](https://github.com/jhu-dvrk/dvrk_model)
- [dVRK ROS 2 software documentation](https://dvrk.readthedocs.io/main/pages/software/ros-2.html)

The simulator reuses dVRK model assets and interface conventions but does not require the dVRK runtime, cisst, or SAW.

