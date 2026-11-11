# Continuous parcel conveyor

The requested behavior supersedes the earlier static/indexed-feed choice for the new
mode. Keep the verified stationary/indexed workflows available.

1. Add a separate continuous mode, initially 0.06 m/s (versus indexed 0.035 m/s).
   Give its three cartons 1.2 m pitch on an extended upstream belt: 20 simulated
   seconds between arrivals. The existing tote still holds three parcels per batch.
2. Keep perception and the belt heartbeat running in a separate ROS executor through
   approach, grasp, placement and verification. Stop on loss of command/perception,
   controller failure or process exit; do not stop for normal picks or placements.
3. Select the closest reachable depth detection and validate conveyor demand against
   velocity from successive camera observations. Use the constant demand for prediction
   and following to avoid accumulating pixel quantization noise. Align horizontally
   above the parcel before descending; execute a sampled, collision-checked following
   path. Close the fingers while following; verify actual total finger gap and retention
   after lifting before placing (the independently driven fingers can see-saw).
4. Independently record Gazebo belt commands and carton poses. Require continuous
   nonzero command through every pick/place plus physical transport, actual upright
   placements and fresh RGBD detections. Reject missed/late intercepts explicitly.
5. Run offline checks, two fresh continuous runs, GUI validation and an indexed
   regression. Document the measured speed, timing, limits and self-test command.

Changes and results are recorded in the shared HANDOFF.md / SESSION_LOG.md. Work stays
in the mirror; no promotion, commit or publication is part of this request.

## Implementation and current verification

Implemented continuous mode in `parcel_task.py`, `parcel_continuous.py`,
`parcel_tracking.py`, the parcel launch, generator/world and scored harness.
The independent observer records actual Gazebo belt commands and carton poses.
The continuous verdict also requires physical parcel motion during finger closure;
publishing a nonzero belt command alone cannot pass it.

The continuous detector also publishes `/parcel_cell/moving_detections` over
y=-0.400..0.180 m, so tracking starts upstream of the grasp window. The original
pick and tote streams keep their existing bounds. The robot waits until a reliable
track enters y=-0.160..-0.075 m before scheduling interception.

Moving approach: 0.8 s horizontal alignment at clear height; descend to the moving
parcel by 1.6 s; close over 1.0 s; verify settled total gap at 3.1 s; lift between
3.2 and 4.5 s, with smooth post-lift deceleration in the last 0.4 s. All are simulation
times. Runtime grasps use RGBD detections only,
never Gazebo entity poses. Joint-space samples are checked against the scene's
physical solids; velocity/acceleration budgets are checked before action submission.

Development failures are retained under `logs/parcel_continuous/prototype_*/`:
startup callback exception, noisy short-window velocity prediction, diagonal approach
contact, and an invalid individual-finger equality check. The latter was replaced
with the existing shared primitive's total-gap criterion: unequal finger positions
can still hold a correctly sized carton. No shared motion or calibration changes.
Partly clipped detections at the ROI edge are excluded from velocity fitting.
An uncertain first velocity fit waits for more observations within the reachable
window; it never authorizes a pick. Repeated starts with the old narrow stream
could not always form a reliable track before the interception deadline, which
motivated the dedicated upstream stream. An exact controller interpolation check
also rejected the old abrupt final stop before motion submission; post-lift
deceleration fixes the path while keeping the same acceleration budget.
The first successful development batch used 1.5 m pitch (25 s); measured return to
the next pick approach takes about 16 s, so the final queue uses 1.2 m (20 s).

The test harness keeps an independent ROS perception reader connected before,
during and after motion. Its atomic snapshots must pass file/receipt/source-age
checks before matching to Gazebo truth. A late reader received no messages in
`final_gui_04`, despite passing physical placements; that failed overall verdict
remains recorded. `final_gui_05/run1` passed every gate with the persistent reader.
Run2 caught floating-point subtraction rejecting an exactly 0.7 s old frame;
source age now uses integer nanoseconds. The exact boundary and one nanosecond
beyond it are explicitly checked in `logs/parcel_continuous/timestamp_boundary.json`.
Observer writes are paced at 10 Hz to avoid unnecessary disk activity.

## Continuous-motion verification before the startup refinement

`logs/parcel_continuous/final_gui_06/` contains two fresh GUI runs of the same final
source (including the timestamp correction). Both source/tool manifests matched the
checkout at that stage. Every geometry, camera, fresh ROS perception, actual placement, continuous
transport and operational health gate passed.

| Run | Domain | Actual tote placements | Belt running to FULL | Largest grasp prediction error |
| --- | --- | --- | --- | --- |
| run1 | 88 | 3/3 upright, stable, full footprint | 54.364 sim s | 4.55 mm |
| run2 | 89 | 3/3 upright, stable, full footprint | 54.580 sim s | 5.64 mm |

There were no zero belt commands during normal handling. Independent Gazebo median
free-travel and grasp velocities were approximately 0.060 m/s in all six picks.
Actual final RGBD/overview images were reviewed. Both task logs had no unexpected
operational errors. The existing MoveIt teardown exit 245 still occurs after
`PARCEL_TASK_COMPLETE`; it is accepted only after independent placement/perception
checks pass, and remains a teardown limitation.

The two-package build passed (existing setuptools warning only). All five offline
suite checks passed, including 15 parcel tests; the explicit timestamp-boundary
check also passed. The fresh GUI indexed regression under
`logs/parcel_continuous/indexed_regression/` passed 3/3 actual placements, independent
perception/transport/camera checks and operational health; its source manifest also
matches. Aggregate results are in `logs/parcel_continuous/validation_summary.json`.
Domains 88/89/90 session locks were reusable after exit, and the final host process
audit found no standard ROS/Gazebo processes left running. No primary promotion or
commit was made.

## Startup refinement

The requested startup now overlaps gripper opening with a direct collision-checked
move to the above-belt waiting pose. Continuous mode skips the setup-time sleep and
blocking prime, and omits the intermediate home move. A short warm-up finger command
and verified opening run alongside the arm. Both motions, fresh feedback and an empty
tote must be confirmed before the conveyor starts. Between picks an already-open hand
is checked rather than commanded through another redundant opening stroke.
Other callers retain the original `setup(prime_gripper=True)` default.

The automatic GUI command has `--quick-start` to begin task preparation as soon as
controllers are active. It retains independent after-batch camera, actual placement,
ROS perception, continuous-transport and operational-health verdicts. The full harness
without that option still performs before-batch geometry and perception inspection.
Records explicitly identify which mode was used.

An initial quick-start run stopped on a genuinely stale 0.75 s camera frame; its
failure is preserved under `logs/parcel_startup/quick_gui_01/`. The freshness limit
was kept. Region detection now shares one depth deprojection per frame and caches
duplicate frame stamps without changing their source timestamps. A 25-pair benchmark
on a 640x480 test frame produced identical coordinates in all three regions and
reduced median computation from 43.9 ms to 20.0 ms. Evidence:
`logs/parcel_startup/perception_benchmark.json`.

Both fresh GUI runs in `logs/parcel_startup/quick_gui_02/` passed 3/3 actual placements,
fresh independent ROS perception, all camera captures, physical continuous transport
and operational health. Their source/tool manifests match the final source.

| Run | Task entry to feed | MoveIt ready to feed | Arm/open overlap |
| --- | --- | --- | --- |
| run1 | 9.76 wall s | 8.95 wall s | 4.86 wall s |
| run2 | 9.00 wall s | 7.84 wall s | 4.75 wall s |

Saved baseline MoveIt-ready-to-feed times were 26.43/31.38 wall s on this host. Logs
confirm no intermediate home before the first pick. Controller/simulator initialization
still precedes task execution. Quick-start mode explicitly skips before-batch inventory;
independent after-batch verdicts are retained. The normal full-audit mode remains available.
Offline suite checks and the final diff/reference review pass. Locks 88/89 were reusable,
and the final host ROS/Gazebo process audit was empty. Aggregate evidence:
`logs/parcel_startup/validation_summary.json`. Existing post-completion teardown 245
remains; no fresh legacy simulation result is claimed for this startup refinement.
