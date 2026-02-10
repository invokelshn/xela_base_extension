# xela_ah_r2c_bringup

Bringup package for Allegro Hand + XELA tactile model visualization on ROS 2.

## What This Package Does

- Loads a selected Allegro/XELA Xacro model.
- Publishes TF with `robot_state_publisher`.
- Publishes joint states with `joint_state_publisher_gui`.
- Opens RViz with a predefined display config.

This package focuses on model bringup and visualization. It does not include tactile hardware driver runtime.

## Package Layout

- `launch/xacro_launch.py`: Main launch entry point.
- `urdf/`: Allegro hand + XELA Xacro models and generated URDF snapshots.
- `mesh/ur_adapter.stl`: Adapter mesh for UR-mounted variants.
- `rviz/`: RViz profiles.
- `config/allegro_hand_left_r2c.srdf`: Disable-collision SRDF snippet.
- `scripts/`: Reserved for helper scripts (currently placeholder only).

## Prerequisites

- ROS 2 environment sourced (Humble/Jazzy compatible usage pattern).
- Required runtime tools:
  - `xacro`
  - `robot_state_publisher`
  - `joint_state_publisher_gui`
  - `rviz2`
- Local package dependency:
  - `xela_models` (referenced by Xacro includes)

Example install (Ubuntu/ROS 2):

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-rviz2
```

## Build

From workspace root:

```bash
colcon build --packages-select xela_ah_r2c_bringup --symlink-install
source install/setup.bash
```

## Launch

Basic usage:

```bash
ros2 launch xela_ah_r2c_bringup xacro_launch.py xela_sensor:=x_allegro_left_curved
```

Supported `xela_sensor` values:

1. `allegro_hand_right_curved`
2. `allegro_hand_left_curved`
3. `x_allegro_left_curved`
4. `x_allegro_left_w_ur_adapter`
5. `x_allegro_right_curved`
6. `x_allegro_right_w_ur_adapter`

Optional overrides:

```bash
ros2 launch xela_ah_r2c_bringup xacro_launch.py \
  xela_sensor:=x_allegro_right_w_ur_adapter \
  rviz_config_file:=/absolute/path/to/custom.rviz
```

## Useful Xacro Rendering Checks

Render without launch:

```bash
xacro $(ros2 pkg prefix xela_ah_r2c_bringup)/share/xela_ah_r2c_bringup/urdf/x_allegro_left_curved.xacro > /tmp/x_allegro_left.urdf
```

Render with arguments:

```bash
xacro $(ros2 pkg prefix xela_ah_r2c_bringup)/share/xela_ah_r2c_bringup/urdf/x_allegro_right_w_ur_adapter.xacro \
  taxels:=1 sensor_collision:=0 parent:=world > /tmp/x_allegro_right_adapter.urdf
```

## Notes and Caveats

- `xacro_launch.py` internally chooses `all_sensor_view.rviz` only for `all_parts_of_individual_module`, but that value is not in `xela_sensor` choices.
- `package.xml` currently does not fully reflect all runtime dependencies used by launch/Xacro (`joint_state_publisher_gui`, `rviz2`, `xela_models`).
- `config/allegro_hand_left_r2c.srdf` is an SRDF fragment containing disable-collision entries.

## Document References

- Detailed package analysis and architecture: `PSD.md`
