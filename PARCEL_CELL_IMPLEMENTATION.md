# Panda industrial parcel cell — implementation and results

Implemented in `~/ros_test_ws_mirror` alongside the original colour sorter. The new variant
keeps the Panda, its fixed base, gripper, controllers and calibrated overhead RGBD camera.
It adds a long contact-driven conveyor, cardboard parcels, a blue tote, industrial pedestal,
cabinet, dense fencing, supported fixtures and saved presentation views.

Implementation and local validation are complete. All final queued, perception, placement,
original-sorter regression and sensor/feed fault gates passed.

## Launch and use

Start in a clean shell from this repository; source the same base and overlay in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch twin_action_demo industrial_parcel_cell_bringup.launch.py indexed:=true
# Separate terminal, with the same environment:
ros2 run twin_moveit_scripts parcel_task --indexed
```

Omit `indexed:=true` and `--indexed` for three stationary staged parcels. Task execution is
explicit; launching the scene alone does not start the arm. Use distinct `ROS_DOMAIN_ID`
and `GZ_PARTITION` values for concurrent cells. Three placements fill the tote; empty/reset
or relaunch before the next batch. The existing colour-sorting launch remains available.

The Panda base remains at world z=0; the new floor is at −0.350 m and the deck at +0.295 m.
The pedestal therefore rises 0.350 m from the floor without changing robot/task frames.
Cartons are 80 × 50 × 50 mm, grasped across their 50 mm dimension. The tote has three slots.

## Implemented components

| Paths | Responsibility |
| --- | --- |
| `src/twin_action_demo/worlds/industrial_parcel_cell.sdf` | Industrial geometry, honest support/collision surfaces, native TrackController and three camera views including the original Panda RGBD sensor. |
| `src/twin_action_demo/models/parcel_box.sdf` | Dynamic 50 g cardboard carton, inertia/friction, tape, label and barcode visuals. |
| `tools/generate_parcel_cell.py` | Reproducible primitive assets, with no downloaded meshes/textures. |
| `src/twin_action_demo/launch/industrial_parcel_cell_bringup.launch.py` | New world, stationary/queued spawning, original bridges, parcel detector and feed guard. Only the controller-YAML resource path is replaced in a temporary URDF so it resolves into this checkout; stored robot files are unchanged. |
| `src/twin_moveit_scripts/twin_moveit_scripts/parcel_layout.py` | Dimensions, pick region, tote slots and full-footprint containment. |
| `…/parcel_perception.py`, `…/object_detector_parcel_cell.py` | Calibrated depth-only top-plane segmentation, independent of cardboard/tote colours; separate pick and tote observations. No Gazebo entity poses enter production perception. |
| `…/parcel_cell_scene.py` | All 15 physical static models / 141 solids in MoveIt, carried-carton geometry, occupied slots and checked attachment/detachment. |
| `…/parcel_task.py`, `…/parcel_feed.py` | Feed → stop → fresh stable observation → pick → place → confirmed settling → ready. Bounded retries, full-tote admission stop and command-loss guard. Settling requires advancing camera timestamps; stale/invalid observations inhibit feed. |
| Package `setup.py` / `package.xml`, `README.md`, `tools/run_offline_tests.sh` | Installed executables/dependencies, launch instructions and expanded offline coverage. |
| `tools/check_parcel_cell.py`, `check_detections_parcel_cell.py`, `observe_parcel_transport.py` | Independent Gazebo placements/transport and ROS perception scoring; unmodified native camera frames saved as PNG. |
| `tools/run_parcel_cell_check.py` / `.sh`, `probe_parcel_transport.py`, `probe_parcel_faults.py`, `test_parcel_cell.py`, `make_parcel_review.py` | Isolated repeat checks, fault injection, offline geometry/perception checks and standalone visual comparison. |

The original world/launch, robot description, controller/bridge/calibration files and
`move_to.py` byte-match their original Git versions. No source promotion, commit or push.

## Validation and evidence

| Check | Observed result | Evidence under `logs/parcel_redesign/` |
| --- | --- | --- |
| Original BT baseline, two fresh runs | 3/3 correct trays in both; zero operational health counts | `baseline_host/` |
| Build and package-prefix checks | Both affected packages rebuilt; prefixes resolve into the mirror | Recorded in shared session log; existing setuptools pytest-repeat warning |
| Expanded offline suite | All five suite checks passed, including five parcel-specific tests and compilation | `offline_final_v2.log` |
| Final SDF syntax | World and carton both Valid | `sdf_final.log` |
| Fresh geometry/contact runs | Stable exact parcel set at z≈0.320 m, actual camera frames | `geometry_02/`, `native_capture_check/`; geometry also gated in every full run |
| Stationary handling, two fresh runs | 6/6 actual parcels upright, stable and fully inside the tote; before/after perception passed | `stationary_03/` |
| Final indexed handling | 6/6 placed across two fresh batches; geometry, transport, perception, captures and operational health passed | `indexed_03/` |
| Contact transport / command loss | All three parcels moved +36.61 mm; zero later drift; invalid speeds stopped | `transport_probe_01/` |
| Sensor/input faults | Seven cases passed: empty/timeout, stale, out-of-region, NaN, unknown class, multiple targets and occupied tote; all stopped feed | `faults_final_03/` (final executor replay) |
| Final original BT regression, two fresh runs | 6/6 correct colour trays, zero operational health counts | `colour_final/` |

The first final indexed batch independently measured about 0.283 / 0.685 / 1.088 m of actual
on-belt displacement before lifting. Post-placement perception error was at most 2.55 mm in that run.
The two final runs measured mean simulation-time / wall-time ratios of 0.44 and 0.46
on this machine. Both source manifests match the final implementation byte for byte.
Operational verdicts use live Gazebo entity poses, full rotated footprints, uprightness and
settling, plus one-to-one ROS detection matching; node status lines are not placement proof.

Review the [standalone target comparison](logs/parcel_redesign/review_final.html) or the
[completed-cell capture](logs/parcel_redesign/indexed_03/run1/after/overview.png).
Source manifests, exact poses, detection errors, transport histories and cleanup records
are retained with the run evidence. The final ownership audit is in
`final_process_audit/audit.json`; source preservation hashes are in `source_integrity.json`.
Generated logs/screenshots remain ignored local artifacts.

## Repeat the checks

Each evidence directory must be new. The harness sources only the ROS base and this overlay,
assigns isolated domains/partitions and cleans only processes with verified run markers.
It returns nonzero when any verdict, timeout, task or operational health gate fails.

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc \
  tools/run_parcel_cell_check.sh --mode indexed --runs 2 \
  --logdir /tmp/panda-parcel-new-check
```

Other modes: `geometry`, `stationary`, `transport`, `faults`, and `colour`. Offline checks:
`bash tools/run_offline_tests.sh`. Runtime checks require host ROS/Gazebo networking/rendering.

## Scope and limits

Known-size aligned cartons and three-slot batches are supported. The right feeder is a
supported stationary section; feeder-to-main transfer and moving-target interception are
not implemented. Camera views were rendered headlessly; interactive GUI operation, Docker
and remote CI were not exercised. The proposed 10 mm minimum clearance was
not measured separately; full planning-scene collision checks and actual handling were used.
Sensor/feed faults were injected; physical dropped-grasp and failed-placement faults were
not injected. Those task branches inhibit feed and stop rather than blindly home a held load.

Successful MoveIt runs retain the pre-existing post-completion teardown rc=245. It is accepted
only after the completion marker, independent placement/perception and known teardown log
signature. The known missing-octomap-plugin message is recorded separately. The deliberately
minimal validation environment omits HOME, so Gazebo warns about its default console-file
location; complete console output is retained in `bringup.log`. These are not new motion
success criteria or suppressed crashes.

## Retained development failures

`geometry_01` exposed an unsupported readiness CLI flag. `stationary_01` exposed a void
MoveIt binding return; installation now uses scene read-back. `stationary_02` exposed
MoveIt's detached-object world copy; it is explicitly removed and checked. `faults_final`
was rejected because its first observer was not discovered yet; discovery is now required.
`indexed_01` failed its camera evidence subscription; native sensor capture and saving the
placement verdict first resolved that evidence issue. `indexed_02` safely stopped on stale
perception; executor ownership and source-frame settling were corrected before final repeats.
All failed evidence remains available; none is counted as a passing run.
