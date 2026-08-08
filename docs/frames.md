# Frames and conventions

## 1. Coordinate tree

The initial scene uses this logical frame tree:

```text
world
├── PSM1_base
│   └── PSM1_RCM
│       └── PSM1_tool
└── ECM_base
    └── ECM_RCM
        └── ECM_optical
```

The exact USD prim names may differ, but the logical names are part of the simulator interface.

Each robot's base transform is defined relative to its configured `parent_frame`. The initial source is YAML:

```yaml
base_frame: PSM1_base
parent_frame: world
base_pose:
  position: [0.0, 0.0, 0.0]
  orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

Future releases may obtain the same transform from TF. The frame names and logical tree remain unchanged.

## 2. Units

- Translation: meters.
- Rotation: radians in ROS joint messages and Python APIs.
- Quaternion order: ROS convention `(x, y, z, w)` at the ROS boundary.
- Isaac Sim internal quaternion ordering must be converted explicitly at adapter boundaries.
- Velocities: meters/second and radians/second.

## 3. RCM and insertion

Yaw and pitch are applied at the RCM. Insertion translates along the configured instrument/endoscope shaft axis after the configured adaptor offset.

The positive insertion direction must be verified against the imported `dvrk_model` URDF and recorded in the implementation tests. It must not be inferred from visual appearance.

## 4. Cartesian pose frames

`measured_cp` represents the pose of the active tool frame:

- PSM: the instrument/tool frame;
- ECM: the endoscope optical/view frame.

The ROS frame ID and reference-frame semantics must follow the current dVRK/CRTK ROS 2 interface. Until verified against that interface, the implementation should expose the reference frame as an explicit configuration field rather than hard-code it.

The ECM camera must be attached to `ECM_optical`, not directly to the mechanical adaptor frame. This keeps the rendered camera orientation separate from the mechanical endoscope model.

## 5. Endoscope view

The simulator renders the endoscope perspective from a camera attached to `ECM_optical`. Camera pose follows the kinematic ECM state, including yaw, pitch, insertion, and roll.

The rendering pipeline provides configurable image dimensions, field of view, clipping planes, simulation-time timestamps, and camera calibration information. Stereo output is selected at launch time and uses the same ECM optical pose with a configurable baseline.

ROS image publication uses `image_transport` conventions. Raw image and camera information are the stable base interfaces; compressed transports are optional transport plugins.

The simulator publishes these base topics:

```text
/ECM/image_raw
/ECM/camera_info
```

For stereo, the corresponding base topics are `/ECM/left/image_raw`,
`/ECM/left/camera_info`, `/ECM/right/image_raw`, and
`/ECM/right/camera_info`. The standard `image_transport` compressed plugin can provide JPEG `/compressed` variants. The scene can instead select the native RTSP transport for efficient network video.


The implementation should preserve the option to support dVRK-style left/right topic names later.

## 6. Required tests

Frame tests must verify:

- home pose values;
- zero yaw/pitch orientation;
- positive and negative yaw/pitch directions;
- positive insertion direction;
- PSM and ECM Cartesian poses against known analytical results;
- ROS quaternion conversion;
- camera pose equals the ECM optical-frame pose.
- rendered image timestamp follows simulation time;
- camera intrinsics match `camera_info`;
- image orientation and optical axis match `ECM_optical`;
- image publication has defined behavior when rendering is unavailable.
