# Product Requirements Description (PRD) — `xela_ah_r2c_bringup`

## 1. Summary

`xela_ah_r2c_bringup` provides the URDF/Xacro description, visualization launch, and collision configuration needed to bring up an Allegro Hand fitted with XELA tactile (taxel) sensors, in both sensorized and non-sensorized configurations, for either hand side. It is a **description/visualization bringup package**: it produces a `robot_description` and displays it in RViz with a mock joint-state source. It does not run any tactile hardware driver, ros2_control controller, or MoveIt planning stack — those are the responsibility of downstream packages (e.g. `ur5e_xahr2c_atag_right_common`) that consume this package's Xacro macros.

## 2. Background / Problem Statement

Allegro Hand is used in multiple robot configurations across this workspace (standalone visualization, UR-arm-mounted manipulation, left- and right-hand variants, with and without XELA taxel sensors, with flat or curved fingertip sensors). Each of these combinations previously risked becoming a hand-maintained, duplicated URDF. `xela_ah_r2c_bringup` exists to:

- Centralize the Allegro Hand + XELA sensor geometry as a single, parameterized, reusable Xacro macro pair (`allegro_hand_right_r2c`, `allegro_hand_left_r2c`), so every downstream robot configuration includes the same macro instead of copying URDF.
- Let a robot integrator choose, per instantiation, whether sensors/covers/palm patches/taxel points are present, and which fingertip sensor type is used, without editing the macro source.
- Provide a fast, standalone way (no MoveIt Pro app, no docker) to visually inspect a given hand configuration in RViz while iterating on sensor placement.

This package is also the **architectural template** other XELA hand-bringup packages are expected to follow — most notably `xela_dg5f_bringup` (Tesollo DG-5F hand), whose `x_dg5f_hand.xacro` macro signature and design conventions (unconditional sensor housings, a `taxels` flag that only gates individual taxel dot markers, a `sensor_collision` flag, thin standalone-demo wrappers over a macro-only core file) were deliberately modeled on this package.

## 3. Goals

1. Provide reusable macros (`allegro_hand_right_r2c`, `allegro_hand_left_r2c`) that any arm/robot Xacro can `xacro:include` and instantiate with an explicit `parent` link and mount offset (`x y z rx ry rz`), so the hand is not forced to be a standalone `<robot>` document rooted at a fixed base link.
2. Support independent toggling of:
   - Protective covers on finger links (`covers`)
   - Sensorized vs. non-sensorized phalanx mesh/mass properties (`phalanges`)
   - Fingertip sensor type: flat, curved, or none (`tips`)
   - Palm sensor presence (`palm`)
   - Whether individual taxel points are instantiated at all (`taxels`)
   - Whether taxel points carry collision geometry (`sensor_collision`)
   - Finger-naming scheme, numeric vs. legacy Japanese-named prefixes (`defaultnames`)
3. Support multiple simultaneous hand instances in one URDF via a `sequence` namespace parameter, so link/joint names never collide.
4. Provide ready-to-run demonstration entry points (hard-coded and Xacro-argument-parameterized) that a developer can launch directly with `ros2 launch xela_ah_r2c_bringup xacro_launch.py` for visual verification, including a UR-arm-mounted variant.
5. Ship a disable-collisions SRDF fragment for the sensorized hand's self-intersecting/near-touching links, for reuse by any MoveIt configuration built on top of this hand.

## 4. Non-Goals

- No tactile hardware driver, sensor-stream publishing, or CAN/serial bridge node — this package is geometry/visualization only.
- No ros2_control hardware interface or controller configuration.
- No MoveIt planning configuration (SRDF here is a disable-collisions fragment only, not a complete SRDF document with groups/end-effectors).
- No left-hand equivalent of every right-hand-only artifact (e.g. only `config/allegro_hand_left_r2c.srdf` exists today; a right-hand SRDF fragment is not yet provided).
- Real-time performance tuning of the taxel count (e.g. reducing collision-checked taxels) is left to the caller via `sensor_collision`/`taxels`, not solved inside this package.

## 5. Target Users / Use Cases

- **Robot configuration authors** who need to `xacro:include` an Allegro Hand with XELA sensors into a larger arm+hand robot description (primary consumer today: `ur5e_xahr2c_atag_right_*` packages).
- **Sensor-placement engineers** iterating on taxel/patch mount offsets, who need a fast RViz feedback loop without booting the full MoveIt Pro application stack.
- **Package authors building a new taxel-sensorized hand bringup** (e.g. `xela_dg5f_bringup`) who use this package's macro signature, parameter set, and file-tiering convention (macro-only core → thin hard-coded demo → Xacro-arg-parameterized demo) as the pattern to replicate for a different hand.

## 6. Functional Requirements

| ID | Requirement |
|----|-------------|
| FR-1 | The package SHALL expose `allegro_hand_right_r2c` and `allegro_hand_left_r2c` Xacro macros with an explicit `parent` link parameter and `x y z rx ry rz` mount offset, so a caller can attach the hand anywhere in a larger URDF. |
| FR-2 | The macros SHALL accept a `sequence` parameter that namespaces every generated link/joint name, allowing multiple hand instances in a single robot description without name collisions. |
| FR-3 | The macros SHALL accept independent boolean-like flags `covers`, `phalanges`, `palm`, `taxels`, `sensor_collision`, and a `tips` enum (`'flat'`, `'curved'`, `'default'`), each of which changes only the geometry/mass properties it documents (no unrelated side effects). |
| FR-4 | Setting `taxels:=0` SHALL omit individual taxel point links/joints while still including sensor housing/patch bodies (housings represent real, always-present hardware; taxels are the deformable sensing points inside them). |
| FR-5 | The package SHALL provide at least one launch file exposing an `xela_sensor` selector that renders and displays a chosen top-level Xacro/URDF file in RViz with `joint_state_publisher_gui` for manual joint manipulation. |
| FR-6 | The package SHALL provide a UR-adapter-mounted hand variant (`x_allegro_{left,right}_w_ur_adapter.xacro`) demonstrating non-`world`-rooted attachment. |
| FR-7 | The package SHALL provide a disable-collisions SRDF fragment covering the sensorized hand's near-touching/self-intersecting links, consumable by downstream MoveIt SRDF assembly. |
| FR-8 | Every Xacro file intended for direct `xacro`/`ros2 launch` invocation (as opposed to macro-only include files) SHALL be renderable standalone without additional undocumented arguments. |

## 7. Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NFR-1 | Rendering any supported top-level Xacro file with `xacro` SHALL succeed with zero parse errors given only the documented arguments. |
| NFR-2 | `package.xml` SHOULD declare every package this package's launch files and Xacro includes actually require at runtime (currently a known gap — see PSD/Design doc). |
| NFR-3 | The macro design SHOULD favor toggles that can be flipped without editing macro source, over hard-coded per-use-case duplication, wherever the underlying geometry is genuinely shared. |
| NFR-4 | Generated `.urdf` snapshot files checked into `urdf/` SHOULD be clearly marked as build artifacts (auto-generated header) and SHOULD NOT be hand-edited. |

## 8. Assumptions and Dependencies

- Assumes the sibling package `xela_models` is present on the Xacro include path and provides `materials.xacro`, `taxel_n.xacro`, `uSPa44.xacro`, `uSPa46_2409.xacro`, `uSCuAH.xacro`, `uSFtAH.xacro`, `aftdn.xacro`.
- Assumes `joint_state_publisher_gui` and `rviz2` are installed on the system even though `package.xml` does not currently declare them (see Section 11, Known Gaps).
- Assumes downstream consumers (e.g. MoveIt Pro robot configuration packages) will wrap this package's macros with their own `robot_description.urdf`/`.srdf`, `ros2_control` configuration, and planning setup — none of that is provided here.

## 9. Success Criteria

- A new robot configuration package can add a fully-sensorized right or left Allegro Hand by writing a single `xacro:include` + macro call, with zero copy-pasted finger/palm/sensor URDF.
- `ros2 launch xela_ah_r2c_bringup xacro_launch.py xela_sensor:=<any documented value>` opens RViz showing the correct hand geometry with manually-movable joints, for all six documented `xela_sensor` choices.
- Toggling `taxels`, `sensor_collision`, `covers`, `phalanges`, `palm`, or `tips` on a running Xacro render produces the documented geometry change and no others.
- A reviewer implementing a new hand's bringup package (e.g. DG-5F) can use this package's macro signature and file layout as a template without needing to reverse-engineer additional undocumented conventions.

## 10. Out-of-Scope Follow-ups (tracked, not committed)

- Declaring the full effective runtime dependency set in `package.xml`.
- Fixing the unreachable `all_sensor_view.rviz` selection branch in `launch/xacro_launch.py` (compares `xela_sensor` against a value, `all_parts_of_individual_module`, that is not among the declared launch-argument choices).
- Providing a right-hand disable-collisions SRDF fragment to match the existing left-hand one.
- Wrapping the SRDF disable-collisions fragment into a complete, groups/end-effector-bearing SRDF if MoveIt planning on this package's output is intended directly (today, planning SRDFs are assembled by downstream packages).

## 11. Known Gaps (carried over from prior package analysis)

1. **RViz selection branch mismatch** — `xacro_launch.py` only selects `all_sensor_view.rviz` when `xela_sensor == 'all_parts_of_individual_module'`, a value absent from the declared `xela_sensor` choices, so this branch never fires; the default `urdf_rviz2.rviz` is always used unless `rviz_config_file` is passed explicitly.
2. **Dependency declaration gap** — `package.xml` declares `joint_state_publisher` but the launch file actually uses `joint_state_publisher_gui`; `rviz2` and `xela_models` are used but not declared at all.
3. **SRDF fragment, not a full document** — `config/allegro_hand_left_r2c.srdf` is a bare list of `<disable_collisions>` entries, not a self-contained SRDF with `<robot>` root/groups/end-effectors; treat it as an include fragment.
4. **Left/right asymmetry in shipped artifacts** — only a left-hand SRDF fragment exists; no right-hand equivalent is shipped from this package today.
