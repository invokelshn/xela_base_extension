# PRD: xela_taxel_msgs

## 1. Purpose

`xela_taxel_msgs` defines ROS 2 message and service interfaces for XELA taxel data
aggregation and runtime visibility control used by visualization and integration nodes.

## 2. Goals

- Provide a stable schema for taxel module arrays.
- Provide temperature-enabled and non-temperature variants.
- Provide service contracts for module/group enable-disable filtering.
- Keep the package as pure interface-only (`rosidl`) with no runtime node logic.

## 3. Non-Goals

- Sensor transport, CAN communication, or device control.
- Filtering algorithm implementation.
- Visualization UI implementation.

## 4. Scope

### 4.1 Messages

- `XTaxelSensor`
- `XTaxelSensorArray`
- `XTaxelSensorT`
- `XTaxelSensorTArray`

### 4.2 Services

- `ToggleModule`
- `ListModuleFilters`
- `ClearModuleFilters`
- `ToggleGroup`
- `ListGroups`
- `ClearGroups`

## 5. Functional Requirements

### 5.1 Message Contracts

- `XTaxelSensor` and `XTaxelSensorT` must include:
  - module identity and metadata (`model`, `sensor_pos`, `frame_ids`)
  - per-taxel values (`xela_server_ros2/Taxel[]`)
  - per-taxel forces (`xela_server_ros2/Forces[]`)
- `XTaxelSensorT` must additionally include `temps`.
- Array messages must wrap module lists and include `std_msgs/Header`.

### 5.2 Service Contracts

- Module-level filtering:
  - toggle by module key (`ToggleModule`)
  - query enabled/disabled runtime lists (`ListModuleFilters`)
  - clear all runtime module filters (`ClearModuleFilters`)
- Group-level filtering:
  - toggle by group name (`ToggleGroup`)
  - list defined and enabled groups (`ListGroups`)
  - clear all group overrides (`ClearGroups`)

### 5.3 Compatibility

- Target ROS 2 distribution: Humble.
- Interface package must build with `ament_cmake` and `rosidl_default_generators`.

## 6. Non-Functional Requirements

- Interface names and fields must remain backward compatible unless version-bumped.
- Build must be deterministic and generate runtime typesupport successfully.
- Service response fields must support human-readable operator feedback (`message` fields).

## 7. Dependencies

- `std_msgs`
- `xela_server_ros2` (for `Taxel` and `Forces`)
- `rosidl_default_generators`
- `rosidl_default_runtime`

## 8. Acceptance Criteria

- `colcon build` succeeds for `xela_taxel_msgs`.
- `ros2 interface show` works for all 4 messages and 6 services.
- Dependent packages can compile against generated interfaces.

## 9. Risks and Constraints

- Changes to `xela_server_ros2` message definitions may break downstream compatibility.
- YAML string fields in list services (`*_yaml`) depend on consumer-side parsing policy.

## 10. Future Considerations

- Add explicit semantic versioning policy for interface-breaking changes.
- Add examples package/tests validating service payload conventions.
