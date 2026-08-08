# dVRK Isaac Sim implementation plan

This plan organizes cleanup and additions for the current ROS 2 / Isaac Sim
kinematic simulator. It is intentionally written as a working document: items
can be checked, reordered, or annotated as implementation decisions evolve.

## Current baseline

The package currently provides:

- ROS 2 and CRTK-style PSM/ECM interfaces;
- kinematic PSM and ECM models with FK, Jacobian, and IK;
- scene YAML files containing robot frames, instruments, endoscope, and camera settings;
- generated USD assets from `dvrk_model` URDF/Xacro files;
- visual-only USD motion in Isaac Sim;
- mono/stereo ECM camera publication with raw and compressed image topics;
- GUI monitoring and joint/state controls;
- PSM jaw interfaces;
- Cartesian commands expressed in the moving ECM view frame;
- operating-state, info, warning, and error topics.

The current automated tests primarily cover pure-Python configuration and
kinematics. Isaac Sim and ROS graph integration are not yet covered by a
repeatable automated test harness.

## Implementation status

Completed in the current refactor:

- Shared typed simulator and scene configuration loading in `scene.py`;
- shared scene filename/full-path resolution for runtime and launch;
- scene validation for robot entries, duplicate names, robot types, camera mode,
  camera owner, and referenced robot configuration files;
- scene-owned instruments, endoscopes, and camera settings;
- extraction of operating-state and Cartesian-frame utilities from
  `ros_interface.py`;
- 12 passing pure-Python tests and a successful package build.

The version-1 ROS adapter has been structurally refactored; remaining Workstream 2 items are behavior definition and integration coverage.

## Phase 1 — Configuration and scene foundation

### 1.1 Centralize configuration loading

- [x] Create typed configuration objects for simulator and scene settings.
- [ ] Add `schema_version` to simulator and scene YAML files.
- [x] Move scene filename/path resolution into one shared module.
- [x] Make both `run_sim.py` and `run_sim.launch.py` use the same resolver.
- [x] Remove duplicated relative-path handling.
- [x] Preserve support for scene filenames and absolute scene paths.
- [x] Keep instrument, endoscope, camera, robot list, and frames scene-owned.
- [x] Keep renderer, Isaac Sim path, cache directory, duration, and ROS environment simulator-owned.

Acceptance criteria:

- A scene can be selected by filename or absolute path.
- Invalid scenes produce one clear validation error before Isaac Sim starts.
- Launch and direct Python execution resolve the same scene to the same files.

### 1.2 Validate scene documents

- [ ] Validate scene name and schema version.
- [x] Validate robot entries and reject duplicate namespaces.
- [x] Validate robot type against the referenced robot YAML.
- [ ] Validate instrument and endoscope fields.
- [ ] Validate camera mode, owner, dimensions, frame, rate, and stereo baseline.
- [x] Validate that every referenced robot configuration exists.
- [ ] Validate that every configured base frame has a pose when YAML frame mode is used.
- [ ] Add tests for valid, incomplete, and malformed scenes.

### 1.3 Define the asset cache contract

- [ ] Add an explicit cache schema/version.
- [ ] Store source model, instrument/endoscope, converter options, and Isaac Sim version in the manifest.
- [ ] Detect stale generated assets instead of relying only on directory names.
- [ ] Add a cache inspection or validation command.
- [ ] Keep generated assets outside source control.

## Phase 2 — Runtime refactor

### 2.1 Split the Isaac Sim runner

Refactor `scripts/run_sim.py` into focused modules:

- [ ] `scene_loader.py` — scene parsing and validation;
- [ ] `asset_cache.py` — generated USD and kinematics lookup;
- [ ] `simulator.py` — `SimulationApp` lifecycle and timeline;
- [ ] `ros_runtime.py` — component creation and stepping;
- [x] `camera.py` — camera creation and publication; (existing module)
- [x] `isaac_ui.py` — GUI integration.

Keep `scripts/run_sim.py` as a thin entry point.

### 2.2 Make the simulation loop explicit

- [ ] Define one authoritative simulation-step sequence.
- [ ] Step ECM before PSMs when Cartesian view conversion is enabled.
- [ ] Publish all timestamps from one simulation-time source.
- [ ] Add reset, pause, resume, and shutdown handling.
- [ ] Define behavior for zero, negative, and very large time steps.
- [ ] Add deterministic stepping mode for tests and scripted experiments.

### 2.3 Add ROS simulation time

- [ ] Publish `/clock` from Isaac Sim timeline time.
- [ ] Configure simulator nodes to use ROS simulation time.
- [ ] Ensure event messages and periodic state messages use the same clock.
- [ ] Document the clock behavior and startup ordering.

Acceptance criteria:

- `ros2 topic echo /clock` advances with the Isaac timeline.
- State, event, camera, and command-related timestamps are consistent.
- Pausing Isaac Sim pauses simulation time and state updates.

## Phase 3 — Kinematics and visual consistency

### 3.1 Establish the kinematics source of truth

- [ ] Define URDF-manifest kinematics as authoritative when a manifest exists.
- [x] Retain analytical PSM/ECM kinematics as a fallback.
- [ ] Report clearly which implementation is active.
- [ ] Reject incompatible joint names and ordering before simulation starts.
- [ ] Add random-sample FK comparisons between analytical and URDF paths.
- [ ] Add Jacobian and IK regression cases near limits and singularities.

### 3.2 Make USD visual updates manifest-driven

- [ ] Extend the conversion manifest with visual link prim paths.
- [ ] Include joint axes and transform directions in the manifest.
- [ ] Remove hardcoded PSM link-path assumptions from `usd_visual.py`.
- [ ] Support instruments with different visual hierarchies.
- [ ] Validate that every configured visual joint exists in the referenced USD.
- [ ] Produce a clear warning or error when visual motion cannot be applied.

### 3.3 Improve asset conversion

- [ ] Separate Xacro expansion, manifest generation, USD import, and cleanup into testable functions.
- [ ] Capture converter logs and source metadata.
- [ ] Make cache replacement recoverable where practical.
- [ ] Add a dry-run mode showing expected output paths.
- [ ] Add an explicit converter compatibility check for Isaac Sim 6.0.

## Phase 4 — ROS/CRTK adapter cleanup

### 4.1 Split the ROS adapter

Refactor `ros_interface.py` into focused components:

- [x] operating-state machine;
- [x] message conversion;
- [x] Cartesian frame conversion;
- [x] joint and jaw command handling;
- [x] publishers and QoS definitions;
- [ ] component/node lifecycle.

### 4.2 Standardize naming

- [ ] Review public and internal names for CRTK naming consistency.
- [ ] Use uppercase acronym forms consistently for `CRTK`, `PSM`, `ECM`, `USD`, and `URDF`.
- [ ] Preserve `config` as lowercase because it is not an acronym.
- [x] Use `CRTKPSM`, `CRTKECM`, and `CRTKROSComponent` directly; no compatibility aliases are required for version 1.

### 4.3 Define interface behavior precisely

- [ ] Document accepted joint-name ordering and partial-name behavior.
- [ ] Define `move_*` versus `servo_*` semantics; servo commands should not rely on an internal trajectory generator.
- [ ] Define command timeout behavior.
- [ ] Define operating-state transitions and rejected-transition behavior.
- [ ] Ensure state event topics use transient-local QoS consistently.
- [x] Add QoS constants instead of creating profiles inline.
- [ ] Define jaw limits, units, and mimic behavior in the scene/robot schema.
- [x] Add tests for all command families and state transitions.

### 4.4 Cartesian frame verification

- [ ] Add explicit tests for PSM commands while ECM yaw, pitch, insertion, and roll move.
- [ ] Test both `measured_cp` and `servo_cp` in view coordinates.
- [ ] Test explicit `world` frame commands.
- [ ] Test orientation-only and translation-only commands.
- [x] Document the dVRK view-axis transform with a diagram or table.

## Phase 5 — Camera and sensor interface

- [x] Keep camera mode and geometry in the scene schema.
- [x] Validate camera owner and require ECM when an ECM camera is selected.
- [ ] Use configured camera frequency when creating and publishing frames.
- [ ] Verify mono and stereo camera calibration values.
- [ ] Verify left/right frame IDs and topic names.
- [ ] Decide whether compressed publication remains manual or is delegated fully to `image_transport`.
- [ ] Add image timestamp and frame-consistency tests.
- [ ] Add a low-rate/headless camera test profile.

## Phase 6 — GUI and usability

- [ ] Display measured and setpoint Cartesian pose in the GUI.
- [ ] Display jaw position for PSMs.
- [ ] Show current scene name and renderer.
- [ ] Show command rejection and IK error messages in the GUI.
- [ ] Add reset/home controls with clearly defined semantics.
- [x] Avoid GUI callbacks modifying state outside the normal ROS/command path.
- [x] Add a clean shutdown path when the Isaac window closes.

## Phase 7 — Testing and quality infrastructure

### 7.1 Unit tests

- [ ] Configuration include/merge and schema tests.
- [x] Scene discovery and path-resolution tests.
- [x] Scene camera and per-robot variant tests.
- [x] FK/Jacobian/IK tests for PSM and ECM.
- [ ] URDF-manifest tests including mimic joints.
- [x] Operating-state tests.
- [ ] Cartesian ECM-view conversion tests.
- [ ] Asset-cache naming and invalidation tests.
- [x] Add a repository-local configuration validation command.

### 7.2 Integration tests

- [ ] ROS adapter test with a native ROS 2 node.
- [ ] Operating-state QoS test with late subscription.
- [ ] Joint command round-trip test.
- [ ] Cartesian command round-trip test.
- [ ] Camera topic and compressed-topic test.
- [ ] Optional Isaac Sim test profile for machines with Isaac Sim installed.

### 7.3 Static and packaging checks

- [ ] Add formatting/linting configuration.
- [x] Add local YAML/configuration validation.
- [ ] Defer repository automation until project workflow is established.
- [ ] Add package import checks under Python 3.12.
- [ ] Add Apache-2.0 license file.
- [ ] Replace placeholder maintainer metadata.
- [ ] Decide whether documentation should be installed with the package.

## Phase 8 — Frame providers and additional devices

- [ ] Define a frame-provider interface.
- [ ] Keep YAML frame provider as the deterministic default.
- [ ] Add TF-based frame provider with timeout and stale-transform handling.
- [ ] Document precedence when both YAML and TF are configured.
- [ ] Add support for additional PSM instruments through scene-only changes where possible.
- [ ] Add optional anatomy/task geometry without coupling it to the patient-cart robot models.
- [ ] Keep dynamics and full patient-cart CAD out of the core scope unless a separate requirement is approved.

## Agreed implementation order

The initial work should prioritize code structure and behavior preservation:

### Workstream 1 — Configuration and scene boundary

Complete Phase 1 first:

1. Centralize configuration and scene parsing.
2. Add typed configuration objects and schema validation.
3. Remove duplicated scene/path resolution from launch and runtime.
4. Add scene and asset-cache tests.

### Workstream 2 — ROS/CRTK adapter refactor

Complete Phase 4 second:

1. Split `ros_interface.py` into operating state, message conversion,
   Cartesian frame conversion, command handling, and lifecycle modules.
2. Preserve the current public topic behavior during the refactor.
3. Standardize CRTK/PSM/ECM naming, without compatibility aliases.
4. Add focused tests for state, joint, jaw, Cartesian, and QoS behavior.

### Workstream 3 — Testing and quality infrastructure

Complete Phase 7 third:

1. Expand unit tests around configuration, scenes, kinematics, and ROS behavior.
2. Add ROS 2 integration tests where the environment is available.
3. Add local YAML validation, formatting, import, and package-build checks.
4. Add optional Isaac Sim integration tests without making Isaac Sim mandatory
   for the pure-Python test suite.

### Deferred work

After these refactors are stable, continue with:

1. Runtime stepping and `/clock` behavior.
2. URDF/USD visual consistency.
3. Camera and GUI improvements.
4. TF frame providers and additional instruments.

## Decisions to record

- [x] Is URDF-manifest FK always authoritative when available? NO — analytical fallback remains supported.
- [x] Use uppercase acronym names directly; no backward-compatible aliases are required for version 1.
- [x] Should camera compressed topics be produced by `image_transport` plugins or by the simulator directly? By `image_transport`.
- [x] Should `/clock` be published by the simulator node or an Isaac Sim bridge extension? Use the Isaac Sim simulator/bridge path.
- [x] What is the required behavior when a scene references an unavailable instrument? Fail to start.
- [ ] What minimum Isaac Sim integration test should run in automated checks?
- [ ] Should scene files support inheritance/includes, or remain self-contained?
