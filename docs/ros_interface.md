# ROS 2 interface contract

This document defines the initial interface surface. Exact message types and field semantics must be checked against the current dVRK ROS 2 implementation before version 1.

The same names are used by the internal Python interfaces. The ROS adapter is a transport layer, not a naming translation layer.

The first ROS 2 adapter is a Python node installed as `dvrk_isaac_sim_ros`. It accepts one `robot_config` parameter and is intended to be namespace-remapped per device:

```bash
ros2 run dvrk_isaac_sim dvrk_isaac_sim_ros \
  --ros-args -r __ns:=/PSM1 \
  -p robot_config:=/path/to/config/PSM1.yaml
```

## 1. Namespaces

The default namespaces are:

```text
/PSM1
/ECM
```

The namespace must be configurable so multiple PSMs can be launched independently.

The supported multi-PSM launch profiles are:

```text
PSM1 + PSM2 + ECM
PSM1 + PSM2 + PSM3 + ECM
```

Each PSM publishes under its own namespace. Base-frame configuration is currently loaded from the selected scene YAML profile. TF-based base-frame lookup is reserved for a future release.

## 2. Initial topics

### PSM

```text
/PSM1/measured_js
/PSM1/measured_cp
/PSM1/measured_cv
/PSM1/move_jp
/PSM1/servo_jp
/PSM1/move_cp
/PSM1/servo_cp
```

### ECM

```text
/ECM/measured_js
/ECM/measured_cp
/ECM/measured_cv
/ECM/move_jp
/ECM/servo_jp
/ECM/move_cp
/ECM/servo_cp
```

The initial payloads follow CRTK 1.0:

```text
measured_js   sensor_msgs/JointState (PSM order: yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw)
measured_cp   geometry_msgs/PoseStamped
measured_cv   geometry_msgs/TwistStamped
move_jp       sensor_msgs/JointState
servo_jp      sensor_msgs/JointState
move_cp       geometry_msgs/PoseStamped
servo_cp      geometry_msgs/PoseStamped
operating_state crtk_msgs/OperatingState
state         crtk_msgs/StringStamped
```

The current adapter publishes `measured_js`, `measured_cp`, `measured_cv`, `setpoint_js`, `operating_state`, and `state`, and subscribes to `move_jp`, `servo_jp`, `move_cp`, and `servo_cp`. Six-DOF PSM Cartesian commands use position-and-orientation IK. ECM Cartesian commands remain position-only because its four joints cannot generally satisfy a full six-axis pose. Both `move_cp` and `servo_cp` use `geometry_msgs/PoseStamped`, matching the dVRK ROS bridge and CRTK Python client.

Each arm also publishes diagnostic events using `crtk_msgs/msg/StringStamped`: `/<arm>/info`, `/<arm>/warning`, and `/<arm>/error`. Initialization milestones and accepted operating-state changes are published on `info`; rejected commands are published on `warning`; IK failures are published on `error`. A Cartesian IK failure disables the arm and publishes the resulting operating-state event.

### Endoscope view

The ECM publishes a rendered endoscope view through ROS 2 image transport:

```text
/ECM/image_raw
/ECM/image_raw/compressed
/ECM/camera_info
```

`/ECM/image_raw` is the raw-image base topic. `image_transport` may expose compressed or other transport variants without requiring changes to the simulator camera implementation. Topic names, encoding, and camera profile are configurable.

The raw and JPEG-compressed image topics are published at the configured camera rate (default 30 Hz), independently of the simulation update rate. The image and camera-info messages use the same simulation timestamp and the configured ECM optical frame ID. Set `scene.camera.mode` to `mono`, `stereo`, or `off` in the selected scene YAML. Stereo uses `/ECM/left/...` and `/ECM/right/...` topics.

The current adapter also publishes state topics:

```text
/PSM1/state
/PSM1/operating_state
/ECM/state
/ECM/operating_state
```

## GUI monitor and controls

Non-headless Isaac Sim runs open a `dVRK CRTK Monitor` window. Each configured PSM or ECM has a panel showing its CRTK operating state, homed status, and measured joints. Revolute joints are displayed in degrees; insertion joints are displayed in millimetres.

The joint fields are editable target values. `Apply joint targets` sends them through the same kinematic command path as `move_jp` and obeys the operating-state gate. The operating-state selector and `Home` button use the same state-machine path as the ROS `state_command` interface.

## 3. Time and pause semantics

The Isaac Sim runner uses a fixed, configurable kinematic timestep from
`simulation_rate_hz` in `config/isaac_sim.yaml` (default: 120 Hz). It publishes
`/clock` from that simulation time. All normal CRTK and camera timestamps use
the same source.

When the Isaac timeline is paused, `/clock` stops and robot state does not
advance. Periodic CRTK state messages continue with a zero timestamp, which
means the simulator process is alive but the reported state is not currently
valid. Operating-state and other event messages retain their latched event
semantics and are not emitted merely because the timeline was paused.

## 3. State semantics

`measured_js` reports the current simulated joint positions and velocities, in the configured joint order. Effort is not physically simulated and should either be omitted or clearly reported as unavailable according to the selected message contract.

`measured_cp` reports the active tool pose using the configured Cartesian reference frame.

`measured_cv` reports the corresponding spatial velocity computed from the joint state and Jacobian, not from noisy physics sensors.

Simulated PSMs and ECM start in `ENABLED` with `is_homed=true` and the insertion joint initialized to `0.12 m`. Motion commands are accepted only in `ENABLED`; `DISABLED`, `PAUSED`, and `FAULT` hold the current joint position. The `operating_state.state` and `state.string` fields are kept synchronized.

`state_command` uses `crtk_msgs/msg/StringStamped` and the command is carried in its `string` field. The state publishers use reliable, transient-local QoS, so a late subscriber receives the most recent state event. State is published at startup and after each accepted state command; it is not published on every simulation update.

The supported commands are:

```text
enable       -> ENABLED
disable      -> DISABLED
pause        -> PAUSED
resume       -> ENABLED
home         -> is_homed=true
unhome       -> is_homed=false
fault        -> FAULT
clear_fault  -> DISABLED
```

`home` and `unhome` are logical operations in this kinematic simulator; no physical homing motion is performed. Invalid commands are rejected and leave the state unchanged.

## 4. Command semantics

`move_jp` is a target command. The simulator interpolates to the target.

`servo_jp` is a continuously refreshed command. If no valid servo command is received within the configured timeout, the robot stops or holds according to configuration.

Cartesian commands are implemented through the configured FK/Jacobian and kinematic IK. PSM commands use position and orientation; ECM commands use the reachable position component.

## 5. Compatibility policy

The ROS adapter must keep message conversion separate from robot logic. Each supported interface version should have a named converter so future dVRK/CRTK changes do not require modifying the kinematic core.

Internal classes and methods should use the same CRTK vocabulary: `measured_js`, `measured_cp`, `measured_cv`, `move_jp`, `servo_jp`, `move_cp`, `servo_cp`, `state`, and `operating_state`. Isaac-specific names belong only in the Isaac backend.

Image transport is a separate sensor interface and does not need a CRTK state or command name. Its camera pose is owned by the ECM optical frame.

### PSM jaw interface

PSMs expose a logical one-joint `jaw` interface in radians. Both `jaw/move_jp` and `jaw/servo_jp` accept a `sensor_msgs/msg/JointState` with one position value. The value drives the two generated instrument jaw links using their URDF mimic ratios (+0.5 and -0.5). State is reported on `jaw/measured_js` and `jaw/setpoint_js`.

The default 420006 limits are -0.349066 to 1.39626 radians; commands outside the configured limits are rejected.

### PSM Cartesian reference frame

When an ECM is present, PSM `measured_cp`, `setpoint_cp`, and `measured_cv` are published in the current `ECM_view` frame. The conversion is explicitly FK-based: PSM world FK is transformed into the PSM base frame, then into the current dVRK view frame derived from ECM optical FK (X-left, Y-up, Z-away). Incoming PSM `move_cp` and `servo_cp` commands follow the reverse path—current ECM view frame to PSM base frame to world—before inverse kinematics. This keeps Cartesian teleoperation aligned while the ECM moves. A command with `header.frame_id: world` is accepted as an explicit world-frame diagnostic command.

