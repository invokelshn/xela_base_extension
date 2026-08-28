# Design Document — `xela_dg5f_bringup`

## 1. Purpose of This Document

This document describes *how* `xela_dg5f_bringup` is built: the macro/file architecture, sensor housing/taxel design, the two rounds of hardware-visualization bug fixes that shaped the current mesh and parent-link layout, the left/right hand status, and known limitations. See `docs/PRD.md` for *why* the package exists and what it must satisfy; see `README.md` for day-to-day build/launch instructions. See Section 10 for this package's vendor dependency.

## 2. Runtime Architecture

### 2.1 Runtime graph (`launch/x_dg5f_right_display.launch.py`)

```
description_file (launch arg, default urdf/x_dg5f_right_standalone.xacro)
   │  + taxels, sensor_collision (forwarded as xacro args)
   ▼
xacro  ──▶  robot_description
                  │
     ┌────────────┼──────────────────────────┐
     ▼            ▼                          ▼
robot_state_   joint_state_publisher   rviz2 (-d rviz_config_file)
publisher       OR joint_state_publisher_gui
                (selected by `gui` launch arg)
```

Pure visualization loop: no controller, no hardware, no MoveIt. `gui:=true` (default) uses `joint_state_publisher_gui` for manual joint sliders; `gui:=false` uses the non-GUI `joint_state_publisher`. `rviz:=false` skips RViz entirely (e.g. for a pure `robot_state_publisher`+`joint_state_publisher` TF-only session).

### 2.2 File tiers

Mirrors `xela_ah_r2c_bringup`'s tiering exactly, by design (see that package's `DESIGN.md` §2.2):

1. **Macro-only core** — `urdf/x_dg5f_hand.xacro`. Defines the `x_dg5f_hand` macro only. Not meant to be rendered standalone; always `xacro:include`d by something that also defines a `parent` link.
2. **Standalone demo wrapper** — `urdf/x_dg5f_right_standalone.xacro`. Includes the macro file, defines a free-floating `base_link`, and instantiates `<xacro:x_dg5f_hand parent="base_link" side="right" .../>` with `taxels`/`sensor_collision` exposed as `xacro:arg`s (both default `0`, explicitly chosen to match what a downstream planning `robot_description` actually loads for MoveIt Pro). Its own header comment is explicit about this division of responsibility: *"For attaching the hand to an arm with a real parent link and mount offset, xacro:include x_dg5f_hand.xacro directly and instantiate `<xacro:x_dg5f_hand parent="..." .../>` instead of this file."*

There is currently no third tier (a hard-coded-args demo, or a UR-adapter-combined variant) the way `xela_ah_r2c_bringup` has — this package has one macro-only file and one parameterized standalone wrapper, nothing else. `urdf_backup_20260805/` preserves the pre-refactor state: a single monolithic `x_dg5f_right.xacro` that combined both roles (macro definition and standalone-document root) in one non-reusable file, before the split into today's two-tier structure.

## 3. Macro Signature and Parameter Semantics

```
x_dg5f_hand (params):
  parent
  x:=0.0 y:=0.0 z:=0.0 rx:=0.0 ry:=0.0 rz:=0.0
  side:='right' prefix:='' base_type:='default' with_mount:='true'
  taxels:=1 sensor_collision:=1
```

| Param | Meaning | Notes |
|---|---|---|
| `parent` | Link the whole hand mounts on, via a fixed joint to `${prefix}rl_dg_base_link`. | Required. This is the property that lets an arm Xacro `xacro:include` and instantiate the hand in place, rather than the hand being a fixed-base standalone robot. |
| `x y z rx ry rz` | Mount offset/orientation relative to `parent` (angles in degrees, converted with `radians()`). | |
| `side` | `'right'` (default) or `'left'`, forwarded to the vendor `dg5f_hand` macro. | Vendor macro supports both (`rl_dg_*`/`ll_dg_*` naming). **XELA sensor attachment inside this file is hardcoded to `rl_dg_*` names regardless of `side`** — see Section 6, Known Gap 1. |
| `prefix` | Namespace prefix applied to XELA-side link/joint names (and forwarded to the vendor macro). | Enables multiple hand instances in one URDF. |
| `base_type`, `with_mount` | Forwarded verbatim to the vendor `dg5f_hand` macro. | Not interpreted by this file. |
| `taxels` | Gates only the individual taxel-dot markers inside every sensor housing (fingertip dome, uSPa22/uSPa46 patches). Forwarded to `dg5f_fingertip`/`uSPa22_dg5f`/`uSPa46_dg5f`'s own internal `taxels` param. | Housings are unconditional (see Section 4). |
| `sensor_collision` | Forwarded into every taxel-bearing sub-macro; gates `<collision>` geometry on individual taxel dots. | Off by default in the standalone wrapper for planning/collision-checking cost. |

### 3.1 Instantiation order (inside the macro body)

1. Fixed mount joint `${prefix}x_dg5f_mount_joint`: `parent` → `${prefix}rl_dg_base_link`.
2. Phalanx-length and per-finger axial-shift/standoff properties (see Section 5.2).
3. `xacro:dg5f_hand` (vendor macro) instantiation, with `tip{1..5}_type="xela"` for every finger — a sentinel value the vendor macro does not recognize, so it emits **no vendor tip link** at any fingertip; the XELA `dg5f_fingertip` housing is attached instead, at the same origin the vendor tip would have used.
4. Fingertip sensors ×5 (`dg5f_fingertip`, sequences `f1`–`f5`).
5. Phalange sensors (uSPa22) ×10 — two per finger (proximal + middle).
6. Sensor-controller back covers (`dg_xctrl_cover`) ×10 — mirrors uSPa22 placement, mounted on the dorsal face opposite each uSPa22.
7. Palm sensor (uSPa46) ×1.
8. Hand-back cover (`rl_dg_back_cover`) — a one-off link/joint written inline in `x_dg5f_hand.xacro` rather than via a shared macro, since only one instance exists.

Total sensor housings: 5 fingertip + 10 uSPa22 + 10 xctrl_cover + 1 uSPa46 + 1 back cover = **27 housings**. Total taxel dot markers when `taxels=1`: 5×12 (fingertip) + 10×4 (uSPa22) + 24 (palm) = **124** (`dg_xctrl_cover` never carries taxels — it is purely cosmetic hardware, matching real controller-cover geometry with no sensing surface).

## 4. The "housings always present, `taxels` only gates dots" convention

This is the single most important design invariant in the package, stated verbatim in `x_dg5f_hand.xacro`:

> *"the sensor housings (fingertip dome, uSPa22/uSPa46 pads, xctrl_cover) are physical hardware that is always present on this hand, so they must always be part of the URDF regardless of the taxels flag. `taxels` only controls whether the tiny prismatic taxel-dot markers inside each housing are instantiated ... mirrors how allegro_hand_right_r2c.xacro's `covers`/tip housing links are unconditional while its own `taxels` param only gates the individual uSPa44 taxel dots."*

The practical reason, also stated in the same comment: MoveIt Pro's own planning `robot_description` loads with `taxels=0` specifically to avoid self-collision-checking 124 tiny near-touching dot markers, while a separate `taxels=1` render is published for visualization only, wired up by whatever downstream package assembles the dual light/full model launch. This package's job is only to make both renders derivable from the same macro call with one flag flipped — the dual-model publishing itself is a downstream concern.

## 5. Sensor Macro Design

### 5.1 Per-macro summary

| Macro (file) | Taxel count | Placement method | Notable params |
|---|---|---|---|
| `dg5f_fingertip` (`dg5f_fingertip.xacro`) | 12 | Per-taxel individually fitted: PCA-plane surface normal at each taxel's mesh location, so each taxel's local +Z aligns with the dome's actual curved surface at that point. | `xoff/yoff/zoff`, `txstep/tystep` (present but the actual 12 taxel calls use fully explicit per-taxel x/y/z/rx/ry/rz, not the grid params) |
| `dg_uSPa22` (`uSPa22_dg5f.xacro`) | 4 | Mesh-measured: 4 raised bump tips found via k-means clustering of near-top-Z vertices, expressed in the mesh's own `Rz(90)` reference frame, with a `Rz(mesh_rz-90)` correction applied so the same 4 coordinates stay locked to the physical bumps for any `mesh_rz`. | `mesh_rz` (90 for standard fingers, 0 for thumb), `taxel_rz` (per-taxel TF-only spin) |
| `dg_uSPa46` (`uSPa46_dg5f.xacro`) | 24 | Pure formulaic 4×6 grid: `x = k·tystep − yoff`, `y = j·txstep + xoff`, fixed `z = zoff`. The only patch still using a grid formula rather than mesh-measured positions. | `xoff/yoff/zoff`, `txstep/tystep` |
| `dg_xctrl_cover` (`dg_xctrl_cover.xacro`) | 0 (never) | N/A — purely cosmetic housing, no sensing surface. | `mesh_rz` only |

The difference between "grid formula" (uSPa46) and "mesh-measured/individually-fitted" (uSPa22, fingertip) placement is a direct consequence of surface flatness: uSPa46's palm patch is flat enough for a uniform step grid to land correctly on every bump; uSPa22's smaller pad and the fingertip dome's curvature are not, so those were derived from the actual mesh geometry (k-means bump clustering for uSPa22, per-vertex PCA normal-fitting for the fingertip) rather than assumed to be a uniform grid.

### 5.2 Phalanx-length and axial-shift properties

`x_dg5f_hand.xacro` declares a small set of shared length constants derived from the vendor hand's own joint origins (comment: *"the length of link L is the translation in the origin of the joint whose parent is L"*):

```
len_f1_prox, len_f1_mid       — thumb-specific
len_std_prox                  — proximal phalanx length shared by fingers 2/3/4
len_f234_mid                  — middle phalanx length shared by fingers 2/3/4
len_f5_mid                    — pinky-specific
```

plus a per-finger `xela_axial_shift_f{1..5}_{prox,mid}` fine-tune offset and two standoff constants (`xela_phalange_standoff`, `xela_xctrl_cover_standoff`) controlling how far a sensor sits off the phalanx surface along its mount axis. These are empirically-tuned, visually-verified constants, not computed from CAD — the header comment is explicit that "all of the placements below are first-pass/empirically-tuned via visual feedback in RViz."

## 6. Bug History: the `dg_rock` Sensor-Detachment Fix

This is the most consequential design event in the package's current state, worth recording in full because it changed two independent things that could easily be conflated.

### 6.1 Symptom

At the `dg_rock` teleoperate waypoint (large finger flexion, ~80°+ on multiple joints per finger), the uSPa22 and `dg_xctrl_cover` sensors on the index/middle/ring fingers visually detached from the phalanx surface — appearing to float or rotate away from the mesh they should sit flush against. Thumb and pinky showed no such symptom.

### 6.2 First (partial, non-root-cause) fix: mesh recentering

**Hypothesis**: sensor STL files kept their raw CAD-assembly vertex coordinates (mesh-body-to-origin distances of many centimeters), compensated by an equally large offset in each macro's `<visual>`/`<collision>` origin. A small angular error in that large compensating offset gets lever-arm-amplified into a large visible error at large joint-bend angles.

**Fix applied**: every sensor STL (`dg_x_tip.stl`, `dg_uspa22_base.stl`, `dg_uspa46_base.stl`, `dg_xctrl_cover.stl`, `dg5f_back_cover_right.stl`) was recentered in-place — the formerly-hardcoded compensating origin was baked directly into the mesh's own vertex data via `trimesh apply_transform`+`export`, and the corresponding macro's `<visual>`/`<collision> origin>` was reduced to identity (or, for `uSPa22`/`dg_xctrl_cover`, to a pure in-plane rotation `mesh_rz` with zero translation, since those two housings share one recentered mesh between the standard-finger orientation and the thumb's rotated orientation). Pre-recentering originals are preserved in `mesh/orig_backup/`.

This fix is real, verified, and kept — it removes a genuine amplification risk and is good practice independent of what caused this specific symptom. But it did **not**, by itself, fix the index/middle/ring detachment. A subsequent re-check against the hand's *actual* `dg_rock` joint values (not an approximated reproduction) showed the symptom persisted after the mesh recentering.

### 6.3 Second (root-cause) fix: parent-link correction

**Actual root cause**: the index/middle/ring uSPa22 and `dg_xctrl_cover` housings were parented one phalanx segment further down the vendor hand's kinematic chain than intended. The vendor chain (`dg5f_description`'s `dg5f_right_fingers.urdf.xacro`) is `rl_dg_N_2` (proximal phalanx) → `rl_dg_N_3` (middle phalanx) → `rl_dg_N_4` (distal/tip phalanx) for fingers N=2,3,4. The "prox" sensor had been attached to `rl_dg_N_3` (the middle phalanx) instead of `rl_dg_N_2`, and the "mid" sensor to `rl_dg_N_4` (the distal phalanx) instead of `rl_dg_N_3` — a naming/attachment mismatch present since the file's earlier refactor, not something the mesh recentering could have addressed.

**Fix applied**: `f2/f3/f4`'s uSPa22 and `dg_xctrl_cover` `parent` links were corrected (`rl_dg_N_3`→`rl_dg_N_2` for "prox", `rl_dg_N_4`→`rl_dg_N_3` for "mid"). The associated length constant `len_f234_mid` was also corrected (it had held `0.02123`, a value matching the *old, wrong* parent's segment length, and was corrected to `0.0388`, the real middle-phalanx length read directly from the vendor URDF's joint origins). The `xela_axial_shift_f{2,3,4}_{prox,mid}` fine-tune constants — previously tuned against the wrong parent and length — were reset and re-tuned visually against the corrected geometry using an interactive RViz + `joint_state_publisher_gui` loop, converging on a small (~1 mm, toward the fingertip) adjustment from center.

**f1 (thumb) and f5 (pinky) were deliberately left untouched.** Both were independently confirmed (via isolated single-joint RViz reproduction at the time, and via the corrected `dg_rock` reproduction) to already be correctly attached — the thumb because its kinematic chain and mount convention differ enough that the same segment-index reasoning doesn't transfer 1:1, the pinky because, despite superficially matching the pre-fix f2/f3/f4 code pattern, its actual parent-link assignment was already correct. An early attempt to "fix" the pinky by analogy with f2/f3/f4 (same code shape ⇒ assumed same bug) was reverted once it was clear the pinky had not actually been reported as broken — a reminder that structural code similarity between fingers is not sufficient evidence a fix should apply uniformly across all of them.

### 6.4 Takeaways encoded in this design

- A large, mesh-baked CAD offset living in a joint shared with taxel placement is a standing risk (small angular error × large lever arm at high joint-bend angles) even when it is not the proximate cause of a given symptom — recentering it is good practice on its own merits.
- Bug reproduction for a reported pose must use that pose's *actual* stored joint values, not an approximation — an early debugging pass on this exact issue that used approximated `dg_rock` joint values produced a misleading "no change" result after a real fix had already been applied.
- "This finger's code looks structurally identical to the broken one" is not evidence that finger is also broken — check whether it was actually reported as broken before changing it.

## 7. Documentation Inconsistency to Reconcile

`x_dg5f_hand.xacro`'s own header comment (the "NOTE - mesh recentering" block) still describes the sensor STLs as retaining "their raw CAD-assembly vertex coordinates," instructing readers not to reintroduce a large compensation offset at the joint level. That guidance ("don't put a large offset in the shared joint") remains correct, but its premise is now stale for the five specific meshes recentered on 2026-08-08 (Section 6.2) — those meshes' vertex data itself no longer carries the raw CAD offset; it was baked out. A future edit of this file should update that header note to describe the *current* state (meshes pre-centered; only small standoffs and, for uSPa22/xctrl_cover, a pure rotation live at the joint level) rather than the pre-fix state.

## 8. Left/Right Hand Status

Unlike `xela_ah_r2c_bringup` (which ships fully mirrored, independently-verified left and right macros — see that package's `DESIGN.md` §5), this package has **only a right-hand-verified implementation today**:

- The `side` parameter exists on `x_dg5f_hand` and is forwarded to the vendor `dg5f_hand` macro, which does support left-hand (`ll_dg_*`) naming.
- Every XELA sensor attachment inside `x_dg5f_hand.xacro` is hardcoded against right-hand (`rl_dg_*`) parent link names, regardless of the `side` value passed in.
- There is no `x_dg5f_left_standalone.xacro` yet; the file's own header comment forward-references it as a TODO.
- Practical implication: calling `x_dg5f_hand` with `side='left'` today produces a vendor left-hand skeleton with XELA sensors either unattached, mis-attached, or erroring, depending on whether the referenced `rl_dg_*` link names happen to also exist in a mixed instantiation. It has not been exercised or verified in this state.

Implementing the left hand should decide, before writing code, whether to follow `xela_ah_r2c_bringup`'s duplicated-file-with-mirrored-literals strategy (safer per-hand verification, more files) or keep this package's existing single-macro-with-`side`-parameter strategy and add the missing mirrored constants inside it (less duplication, requires the same offsets to be independently re-derived and visually verified for the left hand rather than assumed symmetric).

## 9. Relationship to `xela_ah_r2c_bringup`

This package's `x_dg5f_hand.xacro` states in its own header that it "mirrors the `xela_ah_r2c_bringup/allegro_hand_right_r2c.xacro` convention ... so this hand can be `xacro:include`'d and instantiated by an arm xacro with an explicit parent link and mount offset." Concretely reused from that reference:

- The macro-only-core / standalone-demo-wrapper file split.
- The "sensor housings unconditional, `taxels` flag only gates individual dot markers" invariant, including the same rationale (avoid self-collision-checking large numbers of tiny near-touching bodies in the planning model).
- A `sensor_collision` flag with equivalent semantics (gate `<collision>` geometry specifically on taxel dots).

Deliberately diverged from that reference:

- A single macro with a `side` parameter, instead of separate `_left`/`_right`-suffixed macro files — chosen to reduce duplication, at the cost of the left hand not yet being implemented/verified (see Section 8), whereas `xela_ah_r2c_bringup`'s duplicated-file approach ships both hands independently verified from the start.

## 10. Vendor Dependency

`xela_dg5f_bringup` wraps exactly one external package: `dg5f_description`, a Tesollo-maintained **vendor** package brought into the workspace via the `dg5f_ros2` git submodule (`xela_base_extension/.gitmodules` registers `dg5f_ros2`, sibling to another vendor submodule, `allegro_hand_ros2`). It sits as a directory *sibling* to this package (`xela_base_extension/{dg5f_ros2/dg5f_description, xela_dg5f_bringup}`), not nested inside it; the only coupling is this package's `<exec_depend>dg5f_description</exec_depend>` in `package.xml` and a `$(find dg5f_description)` xacro include. `dg5f_ros2` also contains three other vendor packages not used by this package: `dg5f_driver`, `dg5f_gz`, `dg5f_moveit_config`.

```
dg5f_ros2/dg5f_description   (VENDOR, git submodule, Tesollo)
  │  dg5f_macro.xacro: dg5f_hand macro — stock kinematics + mesh only, no sensors
  ▼
xela_dg5f_bringup             (THIS PACKAGE)
     x_dg5f_hand.xacro: wraps dg5f_hand, attaches XELA taxel sensors
```

`dg5f_description`'s `dg5f_macro.xacro` provides the `dg5f_hand` macro:
```
dg5f_hand(prefix:='' side:='left' base_type:='default' with_mount:='true'
          tip1_type:='default' ... tip5_type:='default')
```
This generates **only** the stock Tesollo gripper mechanics: the base link, an optional mount link/joint with vendor mesh, the palm base link with vendor inertials/visual/collision, and (delegated to `dg5f_left_fingers.urdf.xacro`/`dg5f_right_fingers.urdf.xacro`) the finger kinematic chain and stock meshes. It has **no XELA sensors, no taxels, nothing tactile** — `tipN_type` only selects between the vendor's own fingertip mesh variants. Everything this package's own PRD/design describes as sensor housings and taxel dots is added entirely by `xela_dg5f_bringup` on top of this vendor output, via the `tipN_type="xela"` sentinel trick (Section 3.1, step 3) that suppresses the vendor's own tip link so the XELA fingertip housing can take its place.
