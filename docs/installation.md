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

## Isaac Sim ROS 2 smoke test

The first Isaac Sim integration is a headless smoke test. It does not load USD robot assets yet; it validates ROS 2, custom messages, simulation time, and PSM/ECM topic connectivity.

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
  "{name: [yaw, pitch, insertion], position: [0.2, -0.1, 0.08]}"
```

The corresponding `measured_js` and `measured_cp` values should move over subsequent simulation steps. The smoke test prints a successful `crtk_msgs/msg/OperatingState` import before entering the simulation loop.
