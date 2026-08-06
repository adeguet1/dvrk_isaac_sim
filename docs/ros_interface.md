# ROS 2 interface contract

This document defines the initial interface surface. Exact message types and field semantics must be checked against the current dVRK ROS 2 implementation before the first compatibility release.

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
measured_js   sensor_msgs/JointState
measured_cp   geometry_msgs/PoseStamped
measured_cv   geometry_msgs/TwistStamped
move_jp       sensor_msgs/JointState
servo_jp      sensor_msgs/JointState
move_cp       geometry_msgs/PoseStamped
servo_cp      crtk_msgs/CartesianServo
operating_state crtk_msgs/OperatingState
state         crtk_msgs/StringStamped
```

The current adapter publishes `measured_js`, `measured_cp`, `measured_cv`, `setpoint_js`, `operating_state`, and `state`, and subscribes to `move_jp`, `servo_jp`, `move_cp`, and `servo_cp`. Cartesian commands currently use position-only IK; orientation-aware control will be added when the controlled PSM/ECM DOFs support it.

### Endoscope view

The ECM publishes a rendered endoscope view through ROS 2 image transport:

```text
/ECM/image_raw
/ECM/camera_info
```

`/ECM/image_raw` is the raw-image base topic. `image_transport` may expose compressed or other transport variants without requiring changes to the simulator camera implementation. Topic names, encoding, and camera profile are configurable.

The image and camera-info messages use the same simulation timestamp and the configured ECM optical frame ID.

The current adapter also publishes state topics:

```text
/PSM1/state
/PSM1/operating_state
/ECM/state
/ECM/operating_state
```

## 3. State semantics

`measured_js` reports the current simulated joint positions and velocities, in the configured joint order. Effort is not physically simulated and should either be omitted or clearly reported as unavailable according to the selected message contract.

`measured_cp` reports the active tool pose using the configured Cartesian reference frame.

`measured_cv` reports the corresponding spatial velocity computed from the joint state and Jacobian, not from noisy physics sensors.

## 4. Command semantics

`move_jp` is a target command. The simulator interpolates to the target.

`servo_jp` is a continuously refreshed command. If no valid servo command is received within the configured timeout, the robot stops or holds according to configuration.

Cartesian command topics are intentionally deferred until joint-space compatibility is validated.

## 5. Compatibility policy

The ROS adapter must keep message conversion separate from robot logic. Each supported interface version should have a named converter so future dVRK/CRTK changes do not require modifying the kinematic core.

Internal classes and methods should use the same CRTK vocabulary: `measured_js`, `measured_cp`, `measured_cv`, `move_jp`, `servo_jp`, `move_cp`, `servo_cp`, `state`, and `operating_state`. Isaac-specific names belong only in the Isaac backend.

Image transport is a separate sensor interface and does not need a CRTK state or command name. Its camera pose is owned by the ECM optical frame.
