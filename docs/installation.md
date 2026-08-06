# Installation and runtime environment

## Supported baseline

The initial target environment is:

- Ubuntu 24.04;
- ROS 2 Jazzy;
- Isaac Sim 6.0;
- Python 3.12;
- `dvrk_model` available in the ROS 2 workspace or through `DVRK_MODEL_PATH`.

Isaac Sim and ROS 2 must use compatible Python and middleware libraries. ROS 2 custom-message packages used from Isaac Sim Python must be built with Python 3.12.

## Single ROS 2 workspace

Use one overlay workspace for this project. Place `crtk_msgs`, `dvrk_model`, and `dvrk_isaac_sim` together under its `src` directory. This gives Isaac Sim one consistent `install` space containing the custom-message Python modules and the dVRK model package metadata.

```bash
mkdir -p /path/to/isaac_sim_ws/src
cd /path/to/isaac_sim_ws/src

git clone https://github.com/collaborative-robotics/crtk_msgs.git
git clone https://github.com/jhu-dvrk/dvrk_model.git
git clone /path/to/dvrk_isaac_sim
```

Do not create a separate overlay just for `crtk_msgs` or `dvrk_model` for the normal workflow.

## ROS 2 workspace dependencies

The sourced ROS 2 environment must provide:

- `rclpy`;
- `crtk_msgs`;
- `geometry_msgs`;
- `sensor_msgs`;
- `std_msgs`;
- `image_transport` for endoscope image transport;
- `dvrk_model`, or an explicit `DVRK_MODEL_PATH`.

Build the workspace with the same Python version used by Isaac Sim:

```bash
python3 --version
# Expected: Python 3.12.x

cd /path/to/isaac_sim_ws
colcon build --symlink-install
```

The build script asks for the Isaac Sim root directory when needed. It accepts either a direct installation directory or a source/build root containing `_build/linux-x86_64/release/python.sh`:

```bash
cd /path/to/dvrk_isaac_sim
export ISAAC_SIM_DIR=/path/to/isaac-sim
./scripts/build.sh
```

For example:

```bash
./scripts/build.sh
# Enter: ~/devel/isaacsim-6.0.1
```

The script expands `~`, searches both locations, resolves the directory containing the executable `python.sh`, and saves it in the git-ignored `config/isaac_sim.yaml`.
It then sources ROS 2 Jazzy and builds the complete workspace with Python 3.12.

### Forcing Python 3.12

If `python3` resolves to another version, do not change the system-wide `python3` symlink with `update-alternatives`; that can break ROS 2 and other system tools. Invoke the 3.12 interpreter explicitly instead:

```bash
source /opt/ros/jazzy/setup.bash

/usr/bin/python3.12 --version
# Expected: Python 3.12.x

cd /path/to/isaac_sim_ws
/usr/bin/python3.12 -m colcon build --symlink-install
```

If `colcon` is not installed as a Python module for 3.12, create a dedicated 3.12 environment that can still see the system ROS packages:

```bash
python3.12 -m venv --system-site-packages /path/to/ros2_py312
source /path/to/ros2_py312/bin/activate
python --version
# Expected: Python 3.12.x

python -m pip install colcon-common-extensions
source /opt/ros/jazzy/setup.bash
cd /path/to/isaac_sim_ws
python -m colcon build --symlink-install
```

For CMake packages in a mixed workspace, the Python executable can also be made explicit:

```bash
python -m colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3.12
```

After building, verify the generated Python message package with the same interpreter:

```bash
source /path/to/isaac_sim_ws/install/setup.bash
/usr/bin/python3.12 -c "from crtk_msgs.msg import OperatingState; print(OperatingState)"
```

The interpreter used to build `crtk_msgs` and other custom ROS 2 messages must match the Python interpreter used by Isaac Sim. Isaac Sim 6.0 uses Python 3.12.

## Required shell setup

Every shell that launches Isaac Sim or communicates with it must source the system ROS 2 installation and the single project overlay:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
```

When using Isaac Sim's bundled ROS 2 libraries, set the ROS distribution and middleware before launching Isaac Sim:

```bash
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

If the simulator is launched from a native ROS 2 installation, use the corresponding ROS 2 environment consistently for all processes. Do not mix incompatible ROS 2 distributions or Python versions.

## dVRK model assets

If `dvrk_model` is in the sourced workspace, the asset pipeline can discover it through the ROS package index. Otherwise configure the repository explicitly:

```bash
export DVRK_MODEL_PATH=/path/to/dvrk_model
```

The simulator uses the Xacro/URDF and mesh files from this repository as the source of truth. Generated USD assets are cached separately; see the [design specification](design.md#51-usd-conversion-and-caching).

## Custom CRTK messages in Isaac Sim Python

`crtk_msgs/msg/OperatingState` is a ROS 2 custom message rather than an Isaac Sim built-in message. When Python code running inside Isaac Sim imports it, the ROS 2 workspace containing `crtk_msgs` must have been sourced before Isaac Sim starts:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash
export ROS_DISTRO=jazzy

cd /path/to/isaac-sim
./python.sh /path/to/dvrk_isaac_sim/scripts/run_sim.py
```

The same rule applies to any future project-specific ROS 2 messages. They must be built for Python 3.12 and be visible through the sourced workspace.

## ROS 2 adapter smoke test

The current backend-independent ROS adapter can be run for one configured component:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash

ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args \
  -r __ns:=/PSM1 \
  -p robot_config:=/path/to/dvrk_isaac_sim/config/PSM1.yaml
```

The Isaac Sim smoke test below uses the same sourced environment to launch the simulator and its in-process CRTK ROS adapter.

## USD asset conversion

The repository does not commit generated USD files. The source of truth remains
the virtual Xacro/URDF and meshes from dvrk_model; conversion is an explicit,
repeatable build step and generated assets are cached under .generated/.

Source the ROS 2 workspace first so Xacro can resolve dvrk_model, then invoke
the converter with Isaac Sim's Python:

    source /opt/ros/jazzy/setup.bash
    source /path/to/isaac_sim_ws/install/setup.bash
    export DVRK_MODEL_PATH=/path/to/dvrk_model  # optional when the package is sourced

    ${ISAAC_SIM_DIR}/python.sh \
      /path/to/isaac_sim_ws/install/dvrk_isaac_sim/share/dvrk_isaac_sim/scripts/convert_dvrk_model.py \
      --model PSM1 --instrument 420006

Use --model PSM2, --model PSM3, or --model ECM for the other virtual
components. The output directory can be overridden with --output-dir; the
default is .generated/isaacsim-6.0. This keeps conversion separate from the
runtime ROS interface and makes it possible to review or regenerate assets
without committing generated files.

By default, the converter removes importer-authored Physics schemas because
the project uses kinematic motion and does not need PhysX rigid bodies. Use
--keep-physics only when experimenting with dynamic simulation.

## Isaac Sim ROS 2 simulation

The launch file starts the configured multi-device kinematic scene, generates missing PSM/ECM conversion artifacts, and validates ROS 2, custom messages, simulation time, and PSM/ECM topic connectivity. The default profile starts PSM1, PSM2, PSM3, and a meshless ECM camera model.

From the same shell where the ROS 2 workspace was sourced:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash

ros2 launch dvrk_isaac_sim run_sim.launch.py
```

The launch file loads the saved Isaac Sim path and starts `${ISAAC_SIM_DIR}/python.sh`. It can also be overridden without rebuilding:

```bash
ros2 launch dvrk_isaac_sim run_sim.launch.py \
  isaac_sim_dir:=/path/to/isaac-sim \
  headless:=true
```

Select the two-PSM or three-PSM scene and choose the ECM camera mode:

```bash
ros2 launch dvrk_isaac_sim run_sim.launch.py \
  scene_config:=/path/to/isaac_sim_ws/src/dvrk_isaac_sim/config/scenes/ECM_PSM1_PSM2.yaml \
  camera:=mono
ros2 launch dvrk_isaac_sim run_sim.launch.py camera:=stereo
```

Mono publishes `/ECM/image_raw` and `/ECM/camera_info`; stereo publishes the corresponding `left` and `right` subtopics. Use `camera:=off` to disable rendering publication.

Alternatively, select one arm and let the launch file generate its USD asset
when the cache is missing:

```bash
ros2 launch dvrk_isaac_sim run_sim.launch.py \
  arm:=PSM1 \
  generated_dir:=/path/to/isaac_sim_ws/.generated/isaacsim-6.0
```

Use `arm:=ECM`, `arm:=PSM2`, or `arm:=PSM3` as appropriate. The selected arm
is the only kinematic component started by this mode. `instrument:=420006` and
`endoscope:=Si_straight` customize conversion defaults. Variant-specific
assets are cached as `PSM1_420006` or `ECM_Si_straight`.

In a second shell, source the same environments and inspect the topics:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/isaac_sim_ws/install/setup.bash

ros2 topic list | grep -E '^/(PSM1|ECM)/'
ros2 topic echo /PSM1/measured_js
ros2 topic echo /ECM/measured_cp
```

Send a joint command from the second shell:

```bash
ros2 topic pub --once /PSM1/move_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [0.2, -0.1, 0.12, 0.0, 0.0, 0.0]}"
```

The corresponding `measured_js` and `measured_cp` values should move over subsequent simulation steps. The smoke test prints a successful `crtk_msgs/msg/OperatingState` import before entering the simulation loop.
