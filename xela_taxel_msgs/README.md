# xela_taxel_msgs

ROS 2 interface package for XELA taxel visualization and filtering workflows.

## Overview

`xela_taxel_msgs` provides custom `msg` and `srv` definitions used to:

- represent grouped taxel modules (with and without temperature),
- carry module metadata (`model`, `sensor_pos`, frame ids),
- control runtime module/group filtering in visualization nodes.

This package does not run nodes. It only generates ROS interfaces.

## Message Interfaces

### `xela_taxel_msgs/msg/XTaxelSensor`

- `uint32 message`
- `float64 time`
- `string model`
- `uint8 sensor_pos`
- `string[] frame_ids`
- `xela_server_ros2/Taxel[] taxels`
- `xela_server_ros2/Forces[] forces`

### `xela_taxel_msgs/msg/XTaxelSensorArray`

- `std_msgs/Header header`
- `string[] md_frame_ids`
- `xela_taxel_msgs/XTaxelSensor[] x_modules`

### `xela_taxel_msgs/msg/XTaxelSensorT`

Same as `XTaxelSensor` plus:

- `float32[] temps`

### `xela_taxel_msgs/msg/XTaxelSensorTArray`

- `std_msgs/Header header`
- `string[] md_frame_ids`
- `xela_taxel_msgs/XTaxelSensorT[] x_modules`

## Service Interfaces

### `xela_taxel_msgs/srv/ToggleModule`

Request:

- `string module` (`model + module_no`, example: `afc15`)
- `bool enabled`

Response:

- `bool success`
- `string message`

### `xela_taxel_msgs/srv/ListModuleFilters`

Request: none

Response:

- `string enabled_runtime_yaml`
- `string disabled_runtime_yaml`

### `xela_taxel_msgs/srv/ClearModuleFilters`

Request: none

Response:

- `bool success`
- `string message`

### `xela_taxel_msgs/srv/ToggleGroup`

Request:

- `string group`
- `bool enabled`

Response:

- `bool success`
- `string message`

### `xela_taxel_msgs/srv/ListGroups`

Request: none

Response:

- `string groups_yaml`
- `string enabled_yaml`

### `xela_taxel_msgs/srv/ClearGroups`

Request: none

Response:

- `bool success`
- `string message`

## Dependencies

- `std_msgs`
- `xela_server_ros2` (for `Taxel` and `Forces` message types)
- `rosidl_default_generators` / `rosidl_default_runtime`

## Build

Workspace: `~/xela_robotics/99_support/00_ext_xmodels_ws`

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src/xela_base_extention --packages-select xela_taxel_msgs --symlink-install
source install/setup.bash
```

## Quick Validation

```bash
ros2 interface show xela_taxel_msgs/msg/XTaxelSensorTArray
ros2 interface show xela_taxel_msgs/srv/ToggleGroup
```

## PRD

Product requirements document: `xela_taxel_msgs.md`
