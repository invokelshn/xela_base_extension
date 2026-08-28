# Product Requirements Description (PRD) — `xela_dg5f_bringup`

## 1. Summary

`xela_dg5f_bringup` provides the URDF/Xacro description and standalone visualization launch needed to bring up a Tesollo DG-5F (5-finger) hand fitted with XELA tactile (taxel) sensors: a 12-taxel dome on every fingertip, a 4-taxel patch pair on the two distal phalanges of every finger, a 24-taxel patch on the palm, and cosmetic sensor-controller back covers. It is a **description/visualization bringup package**, architecturally modeled on the sibling package `xela_ah_r2c_bringup` (Allegro Hand equivalent): it wraps the vendor `dg5f_description` hand kinematics/mesh and attaches XELA sensor geometry, without running any tactile driver, ros2_control controller, or MoveIt planning stack. Those are the responsibility of whatever downstream robot configuration package consumes this one.

## 2. Background / Problem Statement

The workspace needed a taxel-sensorized DG-5F hand usable both as a standalone visual reference (RViz) and as a reusable component inside a larger arm+hand robot description, without duplicating URDF between those two uses. The package went through an architectural refactor (captured in `urdf_backup_20260805/`) from a single monolithic, non-reusable `x_dg5f_right.xacro` document into the current split:

- a macro-only core (`x_dg5f_hand.xacro`) that any arm Xacro can `xacro:include` and instantiate with an explicit `parent` link and mount offset, and
- a thin standalone demo wrapper (`x_dg5f_right_standalone.xacro`) that roots the same macro at a free-floating `base_link` purely for `ros2 launch` + RViz inspection.

This mirrors `xela_ah_r2c_bringup`'s own macro/demo-wrapper split by design (see that package's `DESIGN.md` §2.2 for the pattern being followed) and its convention that sensor **housings** are always-present hardware while individual **taxel dot markers** are a togglable visualization/collision-cost detail.

A second, larger driver of this package's current shape was a real hardware-visualization bug: at large finger-bend angles (the `dg_rock` teleoperate pose, ~80°+ joint flexion), the index/middle/ring uSPa22 and cover sensors visually detached from the phalanx surface. Root-causing and fixing this (documented in the macro files' own 2026-08-08 comments) drove both a mesh-recentering pass (STL vertex data baked to each sensor's own local origin via `trimesh`, replacing large CAD-assembly-offset compensation that lived in the joint origin) and a structural parent-link correction (the index/middle/ring uSPa22/cover sensors had been attached one phalanx segment further down the kinematic chain than intended).

## 3. Goals

1. Provide a reusable `x_dg5f_hand` macro that any arm/robot Xacro can `xacro:include` and instantiate with an explicit `parent` link and `x y z rx ry rz` mount offset — the hand must never be forced to be a standalone `<robot>` document rooted at a fixed base link.
2. Attach, on top of the vendor DG-5F hand skeleton, all XELA sensor hardware as always-present housings:
   - `dg5f_fingertip` — 12-taxel dome on every fingertip (5 instances)
   - `uSPa22` — 4-taxel patch, two per finger (proximal + middle phalanx), 10 instances
   - `uSPa46` — 24-taxel patch on the palm (1 instance)
   - `dg_xctrl_cover` — cosmetic sensor-controller back cover, dorsal side opposite each uSPa22 (10 instances), plus a dedicated hand-back cover
3. Gate the fine-grained taxel dot markers (124 total: 60 fingertip + 40 uSPa22 + 24 palm) behind a single `taxels` flag, independent of housing presence, so a planning `robot_description` can omit 124 tiny near-touching collision bodies while a separate visualization-only render keeps them.
4. Gate taxel dot `<collision>` geometry behind a `sensor_collision` flag, independent of `taxels`, for further collision-checking cost control.
5. Support multiple simultaneous hand instances via a `prefix` namespace parameter.
6. Provide a `ros2 launch` demo (`x_dg5f_right_display.launch.py`) that opens RViz with `joint_state_publisher`/`joint_state_publisher_gui` for manual joint manipulation and visual sensor-placement verification, with launch-arg overrides matching the Xacro's own arguments.
7. Correctly attach every XELA sensor housing to the *intended* phalanx segment of the vendor hand's kinematic chain, and keep each sensor's mesh geometry centered on its own link origin (no large CAD-assembly offset relying on lever-arm-sensitive compensation elsewhere).

## 4. Non-Goals

- No tactile hardware driver, sensor-stream publishing, or CAN/serial bridge node — geometry/visualization only.
- No ros2_control hardware interface, controller configuration, or MoveIt SRDF/planning setup (those live in whatever downstream robot configuration package consumes this one).
- **Left-hand XELA sensor support is not a goal of the current package state** — see Section 11 (Known Gaps); it is explicitly deferred, not silently missing.
- No guarantee that the 5 unused `rl_dg_{1..5}_x_tip.stl` vendor-style meshes in `mesh/` are consumed by anything; they are not cleaned up as part of this PRD.

## 5. Target Users / Use Cases

- **Robot configuration authors** who need to `xacro:include` a taxel-sensorized DG-5F hand into a larger arm+hand robot description.
- **Sensor-placement engineers** iterating on mesh/taxel mount offsets, who need a fast RViz feedback loop (`x_dg5f_right_display.launch.py`, or a hand-rolled isolated `ROS_DOMAIN_ID` + `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2` loop) without booting the full MoveIt Pro application stack.
- **Anyone debugging a reported visual/geometric defect** (e.g. a sensor appearing detached at a specific hand pose) who needs to reproduce the *exact* reported joint configuration (not an approximation) to confirm root cause and fix.

## 6. Functional Requirements

| ID | Requirement |
|----|-------------|
| FR-1 | The package SHALL expose an `x_dg5f_hand` Xacro macro with an explicit `parent` link parameter and `x y z rx ry rz` mount offset. |
| FR-2 | The macro SHALL accept a `prefix` parameter that namespaces every generated XELA-side link/joint name, so multiple hand instances can coexist in one robot description. |
| FR-3 | The macro SHALL accept a `side` parameter (`'right'` default, `'left'` accepted) forwarded to the vendor `dg5f_hand` macro. Until left-hand XELA sensor offsets are implemented (Known Gap, Section 11), `side='left'` is not required to produce correctly-attached XELA sensors. |
| FR-4 | The macro SHALL instantiate all XELA sensor housings (fingertip domes, uSPa22 patches, uSPa46 patch, xctrl covers) unconditionally — independent of the `taxels` flag — because they represent always-present physical hardware. |
| FR-5 | The macro SHALL accept a `taxels` flag that gates only the individual taxel dot markers inside each housing, and a `sensor_collision` flag that gates `<collision>` geometry on those dots, both forwarded into every sensor sub-macro. |
| FR-6 | Every XELA sensor housing SHALL be parented to the phalanx/segment link that physically carries it in the real hardware (e.g. a finger's "proximal" uSPa22 SHALL be parented to that finger's proximal phalanx link, not an adjacent segment). |
| FR-7 | Every XELA sensor mesh's `<visual>`/`<collision>` origin SHALL be at, or centered near, its own link origin (no large CAD-assembly-offset compensation living in a joint shared with taxel placement, since that offset's rotational error is amplified at large joint-bend angles). |
| FR-8 | The package SHALL provide a standalone demo Xacro (`x_dg5f_right_standalone.xacro`) rooted at a free-floating `base_link`, defaulting `taxels`/`sensor_collision` to the same values MoveIt Pro's own planning `robot_description` actually loads (today: both `0`). |
| FR-9 | The package SHALL provide a `ros2 launch` entry point that renders the standalone Xacro and displays it in RViz with a togglable joint-state source (`joint_state_publisher` vs. `joint_state_publisher_gui`) and a togglable RViz launch. |

## 7. Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NFR-1 | Rendering the standalone Xacro with `xacro` SHALL succeed with zero parse errors for both `taxels:=0` and `taxels:=1`. |
| NFR-2 | Bug reproduction and fix verification for reported visual/geometric defects SHALL use the exact joint values from the actual reported configuration (e.g. a named waypoint's stored `joint_state`), not an approximated/reconstructed configuration. |
| NFR-3 | A mesh's own STL vertex data SHOULD be centered on its link's own origin wherever practical, rather than relying on a large compensating offset in a joint that also carries taxel placements — to bound how much a small offset error is amplified at large joint-bend angles. |
| NFR-4 | This package's own file-tiering convention (macro-only core vs. thin standalone demo) SHOULD stay consistent with `xela_ah_r2c_bringup`'s equivalent convention, since that package is this one's explicit architectural reference. |

## 8. Assumptions and Dependencies

*(See `DESIGN.md` §10 for the vendor dependency in full detail, with exact paths.)*

- Assumes `dg5f_description` (a Tesollo-maintained **vendor** package, brought in via the `dg5f_ros2` git submodule, sibling to this package under `xela_base_extension/`) is present and provides `dg5f_macro.xacro`'s `dg5f_hand` macro, including right-hand (`rl_dg_*`) and left-hand (`ll_dg_*`) link/joint naming. This vendor package provides stock hand kinematics/mesh only — no sensors, no taxels; every sensor housing and taxel described in this PRD is added by `xela_dg5f_bringup` itself.
- Assumes `xacro`, `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `ros2launch` are available (all declared in `package.xml`).
- Assumes downstream consumers provide their own `robot_description.urdf`/`.srdf`, `ros2_control` configuration, planning setup, and dual light/full model launch wiring — none of that is provided here.
- Assumes `xacro`, `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `ros2launch` are available (all declared in `package.xml`).
- Assumes downstream consumers (MoveIt Pro robot configuration packages) provide their own `robot_description.urdf`/`.srdf`, `ros2_control` configuration, planning setup, and the dual light/full (`taxels=0` planning model / `taxels=1` `/robot_description_full` visualization model) launch wiring — none of that is provided here.
- Assumes any downstream package that keys off this package's sensor housing link names (e.g. `f2_dg5f_ft`, `f3_prox_uSPa22`, `palm_uSPa46`) by name/string convention rather than a `package.xml` dependency will notice if those names change — this package does not track or enumerate who those consumers are.

## 9. Success Criteria

- A new robot configuration package can add a fully-sensorized right-hand DG-5F by writing a single `xacro:include` + `x_dg5f_hand` macro call, with zero copy-pasted finger/palm/sensor URDF.
- `ros2 launch xela_dg5f_bringup x_dg5f_right_display.launch.py` opens RViz showing correct hand geometry with manually-movable joints, for both `taxels:=0` and `taxels:=1`.
- At the specific hand pose that previously reproduced the index/middle/ring sensor-detachment defect (the `dg_rock` waypoint's exact joint values), all sensor housings on all five fingers stay visually flush against their phalanx surface.
- Toggling `taxels`/`sensor_collision` on a running Xacro render changes only the taxel dot markers' presence/collision geometry, never the housing geometry or hand kinematics.

## 10. Out-of-Scope Follow-ups (tracked, not committed)

- Implementing and visually verifying left-hand (`side='left'`) XELA sensor offsets, and creating the forward-referenced `x_dg5f_left_standalone.xacro`.
- Reconciling the stale "STL files keep their raw CAD-assembly vertex coordinates" note in `x_dg5f_hand.xacro`'s own header comment with the fact that the individual sensor mesh files were subsequently recentered in-place (2026-08-08) — see `DESIGN.md` §7 for the specific inconsistency.
- Deciding the fate of the 5 unused `rl_dg_{1..5}_x_tip.stl` meshes and the dead-code `taxelntest` macro in `taxel.xacro` (references an undefined `taxel_n` macro).
- Deciding whether `urdf_backup_20260805/` and `mesh/orig_backup/` should stay installed via `CMakeLists.txt`'s wholesale `install(DIRECTORY ...)`, or be excluded from the installed package.
- Writing this package's own explicit, in-repo sensor-numbering convention table, instead of leaving the finger/module naming convention undocumented here.

## 11. Known Gaps

1. **Left hand not functional** — `side='left'` is accepted and forwarded to the vendor macro (which does support it), but every XELA sensor attachment in `x_dg5f_hand.xacro` is hardcoded against right-hand (`rl_dg_*`) link names. Passing `side='left'` today does not produce correctly-attached (or possibly any) XELA sensors.
2. **Stale documentation comment** — `x_dg5f_hand.xacro`'s header still describes the sensor STLs as retaining "raw CAD-assembly vertex coordinates," which is no longer accurate for the meshes recentered in-place on 2026-08-08 (see `DESIGN.md` §7).
3. **Unused vendor meshes** — `mesh/rl_dg_{1..5}_x_tip.stl` are not referenced by any Xacro in this package.
4. **Dead code** — `taxel.xacro`'s `taxelntest` macro references an undefined `taxel_n` macro.
5. **No package-level README/docs existed prior to this PRD/design/README set** — this is the first time this package has been documented as a standalone unit rather than only through inline Xacro comments.
