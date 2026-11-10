# Panda industrial parcel cell — repository-adapted plan

Status: implemented and locally validated on 2026-11-09. This document preserves the proposed
design and gates; see `PARCEL_CELL_IMPLEMENTATION.md` for actual changes and results.
Promotion to the primary workspace remains a separate action.

## Recommended outcome

Build a new parcel-cell variant alongside the existing colour sorter. Match the reference's
composition with the existing Panda, a long black belt, brown cartons, a blue tote, a raised
plinth, cabinet, dense fencing, and a saved close three-quarter view. Develop it in three
useful increments: a physically coherent scene, stationary parcel picking, then indexed
conveyance with repeated picking. Preserve the existing colour-sorting and ground-cell paths.

Use the reference's relationships and palette, adapting dimensions to the actual gripper and
verified task region. Keep the robot description, controllers, joint limits, base transform,
functional RGBD camera and calibration unchanged unless a demonstrated compatibility defect
requires a separately explained change.

## Evidence and current implementation

Read: `/home/pmkhoi/Downloads/Panda_Gazebo_Environment_Redesign_Spec.md`, repository guidance,
handoff/history, `README.md`, `V2_PROGRESS.md`, relevant `V2_RESUME.md` and `V2_DECISIONS.md`
sections, and the source files below. The spec's relative `images/` paths were not found;
the locally available matching visual references were inspected directly:

- Target: `/home/pmkhoi/Downloads/Simulation_of_pick_and_place_robot_application.jpg`.
- Current views: `/home/pmkhoi/Downloads/Screenshot from 2026-11-08 23-33-53.png`,
  `23-33-58.png`, and `23-34-12.png` (same directory and filename prefix).

The current screenshots show a short staging shelf, small trays, sparse fencing and distant
or obstructed camera views. The target instead fills the view with equipment and uses
conveyor length, dense bars, shadows and the tote to establish scale. Its Epson robot and
large parcels cannot be copied directly onto the current Panda application.

| Actual source | Consequence for the redesign |
| --- | --- |
| `src/twin_action_demo/worlds/industrial_sorting_line.sdf` | `pick_belt` is static: its deck is only 0.080 × 0.360 m, with top z=0.295. No transport system is configured here. |
| `src/twin_action_demo/launch/industrial_sorting_line_bringup.launch.py` | Spawns `panda` at z=0 and three coloured boxes at x=0.560, y=+0.110/0/−0.110. World name is `empty`. |
| `src/twin_description/urdf/panda.urdf` | Fixed world-to-base transform; world equals `panda_link0`. RGBD camera is fixed at (0.4,0,1.2). |
| `src/twin_moveit_scripts/twin_moveit_scripts/move_to.py` | Nominal opening is 0.040 m per finger, about 0.080 m total. Top-down grasp orientation is fixed. Heights are relative to object/target z. |
| `…/object_detector_sorting_line.py` | HSV colour classes, fixed 0.025 m half-height, z=0.320 gate, area bounds and limited ROI. Brown appearance does not preserve colour classification. |
| `…/sort_task_sorting_line.py`, `…/sort_tree_sorting_line.py` | Colour routing, tray exclusion and expected grasp width are specific to the present scene. |
| `…/sorting_line_scene.py` | MoveIt models tray walls only; deck/table are deliberately omitted. Existing success does not establish collision coverage for a larger cell. |
| `…/sorting_validator.py`, `tools/check_sorting_line.py`, `tools/check_detections_sorting_line.py` | Hardcoded classes, counts, footprints and heights; some new environment names are explicitly rejected as legacy by the old checker. Use new parcel verdicts. |
| Both packages' `setup.py` | Existing world/launch/model SDF globs cover flat files. Nested meshes, textures and GUI resources need explicit installation support. New nodes need console entry points. |

Installed package metadata confirms Jazzy packages: `ros-jazzy-gz-sim-vendor` 0.0.13,
`ros-jazzy-ros-gz-sim` 1.0.24, and `ros-jazzy-moveit-py` 2.12.4. Current launch selects
Gazebo Harmonic and the world uses Ogre2. Inspect installed interfaces before selecting a
conveyor implementation; no plugin availability is assumed by this plan.

Historical evidence records two successful colour-sort runs and a legacy regression pass.
These are context, not new validation of this checkout. No build or simulator was run for
this planning task.

## Adapt the specification to this robot

| Specification proposal | Recommended adaptation |
| --- | --- |
| 0.78 m belt and 0.35 m raised robot base | Keep Panda at world z=0 and active deck at z=0.295. In the new variant, first evaluate a floor at z=−0.350 and a plinth from that floor to z=0. This gives a 0.350 m pedestal and 0.645 m floor-to-belt height without changing task frames. Extend support legs accordingly. |
| 4.00 × 0.60 m main conveyor | Start with a 2.4–3.0 m long belt along world Y, around 0.18–0.24 m wide along X. Keep its reachable window near x=0.56, y=0. Extend length for composition; do not make the whole belt a pick region. |
| 0.12–0.20 × 0.09–0.17 m carton footprint | Start with the existing 0.040 × 0.040 × 0.050 m payload given a cardboard skin. Then test an aligned 0.080 × 0.050 × 0.050 m carton, with its 0.050 m side across the fingers. The proposed 0.09–0.17 m minimum widths exceed nominal opening. |
| Large tote with 0.22 m walls | Probe a smaller open tote first, about 0.24 × 0.22 m outside footprint and 0.08–0.10 m wall height. Validate rim clearance, open-finger envelope and practical capacity before scaling up. |
| Varied parcel yaw and sizes | Start with aligned, known-size dynamic parcels. Admit variation only after orientation-aware grasping and size-dependent checks are implemented and tested. |
| Blue tote and richer lighting | Keep these in the new variant with parcel-specific perception. Retain the existing colour world's materials and sun-only lighting. Recheck detection after every material/light revision. |
| Reference camera angle | Save an independent presentation view. Leave the calibrated overhead RGBD sensor untouched. |

The lowered floor is a candidate, not a workaround that conceals interpenetration. The
pedestal top must meet the actual mounting plane; its collider must not enclose robot links.
Update cabinet/fence origins, feet and all supports to the new floor. Check the cell from
side and top views as well as the final camera. If it fails, retain the original floor and
use a lower mounting platform; do not lift the fixed robot casually.

All coordinates below use the existing world frame. For a conceptual layout matching the
spec, local conveyor direction +X can map to world +Y, and local rear +Y to world −X.
With the lowered floor, z_local = z_world + 0.350 (equivalently,
z_world = z_local − 0.350). Camera-space left is determined by the saved view, not by a
sign convention.

Candidate tote centre: search around world x=0.32, y=±0.30, with inner floor top near
z=0.295. Choose the side that appears image-left from the final camera. This is outside
some present detector/validator bounds and is NOT covered by past reach results. Require
clear separation from belt, pedestal and approach corridors before accepting any pose.
Fence and cabinet positions must follow the checked robot sweep, with room for belt entry;
do not simply increase existing fence heights in place.

## Architecture and scope

- Preserve the existing world, launch, coloured models, detector, sorter and checker defaults.
  Retain stable names such as `panda`, `pick_belt`, `cabinet` and `fence_back` in the new
  variant where useful. Keep world name `empty` to preserve existing bridge/service paths;
  isolate runs with distinct ROS domains and Gazebo partitions.
- Add an explicit parcel launch/world and parcel detector/task/checkers. In parcel mode,
  all operational cartons route to `collection_tote_blue`. Colour sorting remains available
  through its existing launch; brown parcels do not acquire invisible colour semantics.
- Separate scene loading from automatic task execution. An incomplete scene must not start
  the old colour task or command motions against unverified geometry.
- Reuse `move_to.py`, the behaviour-tree library and trial database where compatible. The
  existing tree contains colour-specific policy; add a parcel orchestrator rather than
  renaming a class and expecting routing to work. Check detection-cache class parsing too.
- Keep a small parcel layout/config shared by spawning, collision geometry and task targets.
  Independently check live Gazebo poses against expected bounds; a common config alone is
  not proof that the actual world matches it. Avoid a broad refactor of the working sorter.

## Implementation sequence and release gates

| Phase | Concrete work | Required evidence before proceeding |
| --- | --- | --- |
| 0 — Baseline and contracts | Inspect processes; capture current GUI/RGBD views; record controller/actions/topics, frames, package prefixes, robot/config hashes and entity dependencies. Read harnesses. Run offline checks and two isolated fresh colour BT sorts. | Both independent 3/3 placement verdicts, log health and saved baseline images. Investigate any baseline failure before geometry work. |
| 1 — Macro scene | Add new world/launch, lowered floor/plinth candidate, long belt, open tote, cabinet/fence blockout, and reproducible presentation view. Keep task disabled; start with stationary parcels. | Clean load, no missing assets, coherent support/contacts in two fresh starts; top/side geometry review; reference comparison with Panda, belt and tote visible. |
| 2 — Collision and reach | Publish relevant deck, rails, tote, cabinet, fence, supports and pedestal into the parcel planning scene. Include carried-parcel volume for transfers. Probe ready, approach, grasp, lift, rim crossing, release and retract. | Actual Gazebo robot poses agree with commands; proposed clearance margin at least 10 mm around non-contact obstacles, or documented measured justification. Two consistent passes and no observed intersections. Reject unreachable geometry. |
| 3 — Stationary parcel handling | Cardboard skin on the known box; depth-based parcel detector; new single-tote routing, slot allocation and independent parcel checker. Then test the longer 50 mm grip-width carton. | Detection accuracy, verified grasp/lift, release and settled full-footprint containment. Three parcels in three distinct slots, repeated in two fresh runs, with no new controller/planner/traceback failures. |
| 4 — Reference detailing | Rails, rollers, supported feeder, tote rim, carton tape/labels, cabinet slots, dense fence bars, orange ring fixture, conduit, floor markings and shadows. Tune saved camera. | Side-by-side target comparison plus top/side views; repeat parcel detection and placement after final materials/lighting. Record frame rate/real-time factor and simplify details if needed. |
| 5 — Indexed feed | Inspect available Harmonic transport interfaces; prototype real parcel motion on a small test belt, then integrate stop-and-pick control and repeat-cycle recovery. | Ground-truth transport, stopping, picking and tote placement for at least six sequential parcels across two fresh runs. Test faults below; transport and overall verdicts must both pass. |
| 6 — Final regression/package | Offline suite, two fresh parcel runs, two fresh colour BT sorts; legacy ground run if shared motion code changed. Install all resources, document exact launch/test commands and limitations. | Separate visual/physics/operation status table, screenshots, robot/config hash comparison and durable verdict/log paths. Promote to primary only on user authorization. |

Collision treatment is a prerequisite, not visual polish. Do not copy the current blanket
deck/table omission into a broader scene. First exploit the existing fingertip-to-surface
clearance and model real surfaces. If contact requires exceptions, scope them to the
specific fingers/object and manipulation phase using supported MoveIt interfaces; restore
the scene afterward. Confirm the robot and held carton clear rails and rims throughout the
trajectory, not merely at waypoints. New task execution should fail if required collision
geometry cannot be installed. Keep Gazebo collision surfaces physically truthful.

## Parcel perception and repeated handling

Use the current RGBD feeds and calibrated world transform. For the first known-height parcel,
segment points above the measured deck plane inside a deliberately narrow pick ROI; form
components and use the top surface to estimate XY and centre z. Derive object centre from
top height minus half the known parcel height. Do not assume the old saturation, area,
class or fixed centre-height gates apply. Keep tote, queued parcels, arm and labels from
becoming eligible targets through geometry/ROI and observations from a clear ready pose.
Unknown height/orientation needs explicit estimation or rejection, not guessed defaults.

Use Gazebo object identities and poses only in test instrumentation (and explicitly labelled
diagnostic runs), not as the production vision result. Published detections must be fresh,
finite and inside the validated pick region. Initial proposed gate: all eligible targets
matched one-to-one, zero phantoms, XY and centre-Z error at most 10 mm; tighten if measured
grasp margin requires it. Check arm-present, empty-belt and blue-tote-visible cases.

Use distinct tote slots and track capacity. Do not repeatedly drop at the same centre or
count pre-staged decorative cartons as handled output. For the six-parcel indexed test,
use two batches of three with an explicit tote-empty/reset transition, unless a larger
capacity has been independently proved. A full tote stops admission.

Indexed sequence: admit one parcel → transport to the observation window → stop → verify
stable pose and acquire fresh detection → pick → lift/transfer → place/release → confirm
settled placement → return ready → admit next. Use simulation-time stability sampling,
timeouts and an explicit stop/empty/full/fault state. Choose speed and stopping bounds from
measured transport before integrating the robot. This plan does not include moving-target
interception. Pose teleportation is suitable for reset, not acceptance evidence of conveying.

For the right feeder, begin with a supported stationary section. Claim physical transfer
only after testing its contact surfaces, gap and parcel passage. If a compatible transport
system is unavailable, report the scene and stationary picking as complete while leaving
conveyance open; do not substitute texture animation or claim a functioning infeed.

Required fault checks: empty detection, stale/out-of-region detection, belt-stop timeout,
failed/empty grasp, dropped parcel, placement failure and full tote. Hold the feed stopped
on faults, bound retries, and prevent a second parcel entering the active pick zone. Recovery
must account for a possibly held parcel before commanding ready. The old tree's “no pending
detections means clear/done” policy is insufficient while more feed can arrive.

## Proposed files and verification practice

These names are proposals, not existing commands or implemented interfaces:

- `src/twin_action_demo/worlds/industrial_parcel_cell.sdf` and
  `launch/industrial_parcel_cell_bringup.launch.py`.
- New carton models under `src/twin_action_demo/models/`; presentation/layout resources
  under `config/`, with installation rules extended where needed.
- `src/twin_moveit_scripts/twin_moveit_scripts/parcel_cell_scene.py`,
  `object_detector_parcel_cell.py`, `parcel_task.py`, and a feed coordinator only when
  transport is integrated. Register actual executable entry points in the owning package.
- `tools/check_parcel_cell.py`, `check_detections_parcel_cell.py`,
  `reach_probe_parcel_cell.py`, `run_parcel_cell_check.sh` and focused offline checks for
  the new geometry/perception/state policy. Reuse established parsing/math when suitable.

The new harness must return nonzero for failed checker, timeout, load or log-health gates;
inspect the existing harness's printed verdicts because its exit code is insufficient.
Verify exact expected operational parcel IDs/count, rotated full-footprint containment,
settled z and stability over time, and actual displacement for transport. Preserve failure
logs. A historical MoveIt teardown rc=245 is acceptable only after completed work, a passing
independent verdict and evidence it is the same known teardown event.

Build and run in a clean shell with only `/opt/ros/jazzy` and the mirror overlay, verify
package prefixes, and record ROS domain/Gazebo partition and owned processes. The URDF
currently embeds an absolute controller-YAML path into the primary workspace; Phase 0 must
check which configuration is actually loaded. Do not rewrite robot-related launch/URDF code
for this plan; if isolation requires a compatibility fix, document and validate it separately.

For implementation, keep generated screenshots, databases and logs in a fresh local evidence
directory; publish selected artifacts only when requested. Compare final robot-description,
controller, bridge and calibration files to the baseline. Update shared handoff/history at
milestones, leaving historical V2 ledgers intact unless their update is requested.

Original planning audit: reviewed actual sources, four available images, installed package
metadata and document references before implementation. Execution is now complete in the
mirror; `PARCEL_CELL_IMPLEMENTATION.md` records the fresh builds, independent repeated
simulation verdicts, visual evidence, scope adaptations and remaining limitations.
