# Package Specification Document (PSD)

## 1. Package Summary

- Package name: `xela_ah_r2c_bringup`
- Version: `0.0.1`
- Build type: `ament_cmake`
- Primary role: Launch and visualize XELA-enabled Allegro Hand models using Xacro, `robot_state_publisher`, `joint_state_publisher_gui`, and RViz.
- Scope: Description/visualization bringup only. No tactile driver or sensor-stream node is included in this package.

## 2. Architecture Overview

### 2.1 Runtime graph (from `launch/xacro_launch.py`)

1. Launch argument `xela_sensor` selects one Xacro file in `urdf/`.
2. `xacro` is executed to generate `robot_description`.
3. `robot_state_publisher` publishes TF from `robot_description`.
4. `joint_state_publisher_gui` publishes synthetic joint states.
5. `rviz2` visualizes the model and TF tree.

### 2.2 Data flow

`xacro file` -> `robot_description` parameter -> `robot_state_publisher` -> `/tf`, `/tf_static` -> `rviz2`

`joint_state_publisher_gui` -> `/joint_states` -> `robot_state_publisher` -> `rviz2`

## 3. File and Directory Roles

- `launch/`
- `xacro_launch.py`: Main bringup launch file.

- `urdf/`
- `allegro_hand_left_r2c.xacro`, `allegro_hand_right_r2c.xacro`: Core hand macros.
- `allegro_hand_left_curved.xacro`, `allegro_hand_right_curved.xacro`: Simple wrappers for curved-tip defaults.
- `x_allegro_left_curved.xacro`, `x_allegro_right_curved.xacro`: Parameterized wrappers.
- `x_allegro_left_w_ur_adapter.xacro`, `x_allegro_right_w_ur_adapter.xacro`: UR adapter + Allegro combined models.
- `x_allegro_left_no_taxel.urdf`, `x_allegro_right_no_taxel.urdf`: Pre-generated URDF snapshots.

- `mesh/`
- `ur_adapter.stl`: Adapter geometry used by `*_w_ur_adapter.xacro`.

- `rviz/`
- `urdf_rviz2.rviz`: Default hand visualization profile.
- `all_sensor_view.rviz`: Extended sensor-frame visualization profile.

- `config/`
- `allegro_hand_left_r2c.srdf`: Disabled-collision entries (stored as SRDF snippet).

- `scripts/`
- Placeholder only (`.gitkeep`), no runtime script currently installed.

- `resource/`
- Ament index marker file for package registration.

## 4. Supported Launch Models

`xacro_launch.py` allows these `xela_sensor` choices:

1. `allegro_hand_right_curved`
2. `allegro_hand_left_curved`
3. `x_allegro_left_curved`
4. `x_allegro_left_w_ur_adapter`
5. `x_allegro_right_curved`
6. `x_allegro_right_w_ur_adapter`

## 5. Build and Install Behavior

From `CMakeLists.txt`, the package installs:

- `launch/`, `config/`, `urdf/`, `mesh/`, `rviz/` to `share/${PROJECT_NAME}`
- `scripts/` to `lib/${PROJECT_NAME}`
- `resource/xela_ah_r2c_bringup` marker file to `share/${PROJECT_NAME}`

## 6. Dependency Model

### 6.1 Declared in `package.xml`

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `ros2launch`

### 6.2 Effectively required by launch/xacro content

- `joint_state_publisher_gui` (used in launch node)
- `rviz2` (used in launch node)
- `xela_models` (used by multiple Xacro includes)

These effective dependencies should be present in the workspace/system even if not fully declared in `package.xml`.

## 7. Operational Notes and Risks

1. RViz selection branch mismatch:
- Launch computes `all_sensor_view.rviz` only when `xela_sensor == all_parts_of_individual_module`, but that value is not in `xela_sensor` choices.
- Result: default path always resolves to `urdf_rviz2.rviz` unless `rviz_config_file` is manually overridden.

2. Dependency declaration gap:
- Launch uses `joint_state_publisher_gui` and `rviz2`, but `package.xml` declares `joint_state_publisher` only.
- Xacro files depend on `xela_models`, also not declared here.

3. `config/allegro_hand_left_r2c.srdf` is stored as a snippet:
- The file contains many `<disable_collisions ... />` entries without full SRDF root structure.
- Treat it as a reusable fragment unless wrapped by another generator or template.

4. The package is visualization-centric:
- No hardware interface, no tactile stream ingestion, and no ros2_control controller launch in this package.

## 8. Recommended Follow-ups

1. Align declared dependencies with runtime reality (`joint_state_publisher_gui`, `rviz2`, `xela_models`).
2. Fix or remove unreachable RViz selection condition in launch.
3. If MoveIt use is intended, wrap SRDF snippet into a complete SRDF document and document ownership.
4. Keep generated URDF snapshots versioned only when they are intentionally used as release artifacts.
