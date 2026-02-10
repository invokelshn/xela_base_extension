# xela_base_extension

`xela_base_extension` is a collection of ROS 2 packages that extends the base stack
for XELA tactile integration.

Current packages:

1. `xela_ah_r2c_bringup`
2. `xela_taxel_joint_state_publisher`
3. `xela_taxel_msgs`

## Purpose

- Provide visualization bringup for Allegro Hand + XELA models
- Provide custom taxel message/service interfaces
- Provide a ros2_control controller that filters and republishes taxel joint states

## Package Overview

### `xela_ah_r2c_bringup`

- Role: Xacro-based model loading with `robot_state_publisher`,
  `joint_state_publisher_gui`, and RViz bringup
- Nature: simulation/visualization focused package (no hardware driver runtime)
- Key paths:
  - `xela_ah_r2c_bringup/launch/xacro_launch.py`
  - `xela_ah_r2c_bringup/urdf/`
  - `xela_ah_r2c_bringup/rviz/`

### `xela_taxel_msgs`

- Role: ROS 2 interface package for XELA taxel data
- Provided interfaces:
  - Msg: `XTaxelSensor`, `XTaxelSensorArray`, `XTaxelSensorT`, `XTaxelSensorTArray`
  - Srv: `ToggleModule`, `ListModuleFilters`, `ClearModuleFilters`, `ToggleGroup`, `ListGroups`, `ClearGroups`
- Dependency: `xela_server_ros2` message types (`Taxel`, `Forces`)

### `xela_taxel_joint_state_publisher`

- Role: ros2_control controller plugin for filtering/republishing taxel JointState
- Core capabilities:
  - Load `keep_joints` from device profiles
  - Merge/filter source topics
  - Control output topic and publish rate
  - Left/right prefix remapping via `hand_side`
- Plugin type:
  - `xela_taxel_joint_state_publisher/XelaTaxelJointStatePublisher`

## Dependency and Data Flow

1. `xela_taxel_msgs` provides interface contracts (messages/services)
2. `xela_taxel_joint_state_publisher` provides runtime processing (controller)
3. `xela_ah_r2c_bringup` provides model/TF/RViz bringup

In short: interface (`xela_taxel_msgs`) -> processing
(`xela_taxel_joint_state_publisher`) -> visualization/bringup (`xela_ah_r2c_bringup`).

## Build

Workspace root:

```bash
cd <your_ros2_workspace>
source /opt/ros/humble/setup.bash
colcon build --base-paths src/xela_base_extension --symlink-install
source install/setup.bash
```

Example: build a single package

```bash
colcon build --base-paths src/xela_base_extension --packages-select xela_taxel_msgs --symlink-install
```

## Quick Checks

```bash
colcon list --base-paths src/xela_base_extension
ros2 interface show xela_taxel_msgs/msg/XTaxelSensorTArray
```

## Document Links

- `xela_ah_r2c_bringup/README.md`
- `xela_ah_r2c_bringup/xela_ah_r2c_bringup.md`
- `xela_taxel_joint_state_publisher/README.md`
- `xela_taxel_joint_state_publisher/PRD.md`
- `xela_taxel_msgs/README.md`
- `xela_taxel_msgs/xela_taxel_msgs.md`

## Repository Split Notes

- This folder is structured to be split into a standalone git repository.
- If you split it, explicitly document external dependency versions/branches
  such as `xela_server_ros2` and `xela_models`.
