# xela_dg5f_bringup

Bringup package for the Tesollo DG-5F (5-finger) hand with XELA tactile sensors attached: a 12-taxel dome on every fingertip, a 4-taxel patch pair on the two distal phalanges of every finger, a 24-taxel patch on the palm, and cosmetic sensor-controller back covers.

## What This Package Does

- Wraps the vendor `dg5f_description` hand kinematics/mesh and attaches XELA sensor geometry via a reusable Xacro macro.
- Provides a standalone RViz demo for visual inspection of sensor placement.
- Publishes TF with `robot_state_publisher` and joint states with `joint_state_publisher`/`joint_state_publisher_gui`.

This package focuses on model bringup and visualization. It does not include a tactile hardware driver, ros2_control configuration, or MoveIt planning setup — those live in whatever downstream robot configuration package consumes this one.

Beyond standalone visualization, the primary use case is `xacro:include`-ing the `x_dg5f_hand` macro from another robot's Xacro file, so an arm (or any other parent link) can mount a fully-parameterized, sensorized DG-5F hand without duplicating URDF. See `docs/DESIGN.md` for the full macro parameter reference.

```xml
<xacro:include filename="$(find xela_dg5f_bringup)/urdf/x_dg5f_hand.xacro" />
<xacro:x_dg5f_hand
  parent="tool0" side="right" prefix=""
  x="0.0" y="0.0" z="0.0" rx="0" ry="0" rz="0"
  taxels="0" sensor_collision="0" />
```

> **Left hand not yet supported.** `side='left'` is accepted and forwarded to the vendor hand macro, but every XELA sensor in this package is currently attached using right-hand (`rl_dg_*`) link names. See `docs/DESIGN.md` §8 for status.

## Where This Fits

```
dg5f_ros2/dg5f_description   (VENDOR, git submodule, Tesollo — stock kinematics/mesh, no sensors)
        │
        ▼
xela_dg5f_bringup            (THIS PACKAGE — adds XELA taxel sensors on top)
```

See `docs/DESIGN.md` §10 for the vendor dependency in detail. Sensor housing link names generated here (`f{1..5}_dg5f_ft`, `f{1..5}_{prox,mid}_uSPa22`, `palm_uSPa46`) are a public naming surface — some downstream code may key off them by string convention rather than a `package.xml` dependency, so treat renaming them as a breaking change even though `colcon build` won't catch it.

## Package Layout

- `urdf/x_dg5f_hand.xacro`: Core reusable macro — attach a sensorized DG-5F hand to any `parent` link.
- `urdf/x_dg5f_right_standalone.xacro`: Demo wrapper — roots the macro at a free-floating `base_link` for `launch/x_dg5f_right_display.launch.py`.
- `urdf/dg5f_fingertip.xacro`, `urdf/uSPa22_dg5f.xacro`, `urdf/uSPa46_dg5f.xacro`, `urdf/dg_xctrl_cover.xacro`: Individual sensor housing macros (fingertip dome, phalanx patch, palm patch, cosmetic back cover).
- `urdf/taxel.xacro`, `urdf/materials.xacro`: Shared taxel-dot primitive and RViz color materials.
- `launch/x_dg5f_right_display.launch.py`: RViz + joint-state-publisher demo launch.
- `config/x_dg5f_right_display.rviz`: RViz profile for the demo launch.
- `mesh/`: Sensor housing STL meshes (each recentered on its own link origin — see `docs/DESIGN.md` §6.2).
- `mesh/orig_backup/`: Pre-recentering (2026-08-08) original meshes, kept for reference.
- `urdf_backup_20260805/`: Pre-refactor Xacro files (before the macro/standalone-wrapper split), kept for reference.
- `fcstd/`: FreeCAD assembly source used to derive/verify sensor placement (git-ignored).
- `docs/PRD.md`, `docs/DESIGN.md`: Requirements and architecture/design documentation for this package.

## Prerequisites

- ROS 2 environment sourced.
- `dg5f_description` (vendor package providing the DG-5F hand kinematics/mesh).
- Runtime tools declared in `package.xml`: `xacro`, `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`.

## Build

From workspace root:

```bash
colcon build --packages-select xela_dg5f_bringup --symlink-install
source install/setup.bash
```

## Launch (standalone visualization demo)

```bash
ros2 launch xela_dg5f_bringup x_dg5f_right_display.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `description_file` | `urdf/x_dg5f_right_standalone.xacro` | Xacro file to render. |
| `rviz_config_file` | `config/x_dg5f_right_display.rviz` | RViz profile. |
| `gui` | `true` | Use `joint_state_publisher_gui` (sliders) instead of `joint_state_publisher`. |
| `rviz` | `true` | Launch RViz2. |
| `taxels` | `0` | Instantiate individual taxel-dot markers (124 total) inside every sensor housing. Sensor housings themselves are always present regardless of this flag. Default `0` matches what MoveIt Pro's own planning `robot_description` actually loads. |
| `sensor_collision` | `0` | Give taxel-dot markers `<collision>` geometry (only relevant when `taxels:=1`). |

Example with taxel dots visible:

```bash
ros2 launch xela_dg5f_bringup x_dg5f_right_display.launch.py taxels:=1 sensor_collision:=0
```

## Useful Xacro Rendering Checks

Render without launch:

```bash
xacro $(ros2 pkg prefix xela_dg5f_bringup)/share/xela_dg5f_bringup/urdf/x_dg5f_right_standalone.xacro > /tmp/x_dg5f_right.urdf
```

Render with taxel dots and check for parse errors only:

```bash
xacro $(ros2 pkg prefix xela_dg5f_bringup)/share/xela_dg5f_bringup/urdf/x_dg5f_right_standalone.xacro \
  taxels:=1 sensor_collision:=1 > /tmp/x_dg5f_right_full.urdf
check_urdf /tmp/x_dg5f_right_full.urdf
```

## Notes and Caveats

- Left-hand XELA sensor offsets are not implemented — see `docs/DESIGN.md` §8.
- `x_dg5f_hand.xacro`'s own header comment about "raw CAD-assembly vertex coordinates" is stale for the 5 meshes recentered on 2026-08-08 — see `docs/DESIGN.md` §7.
- `mesh/rl_dg_{1..5}_x_tip.stl` are vendor-style per-finger tip meshes not referenced by any Xacro in this package.
- `CMakeLists.txt` installs `urdf/`, `mesh/`, `config/`, `launch/` wholesale, which currently also installs `urdf_backup_20260805/` and `mesh/orig_backup/` as-is.

## Document References

- Requirements / scope: `docs/PRD.md`
- Architecture, macro parameter design, sensor placement methodology, bug-fix history, left/right status: `docs/DESIGN.md`
