# 7 / 8 / 10-parcel continuous batch on the ORIGINAL conveyor

Goal: pick and place more than three parcels per batch **without lengthening the
conveyor**, by shortening the arm cycle and matching belt speed to it. The three-box
profile and the legacy colour sort remain available and unchanged.

Status (2026-11-10): **10 boxes validated** at 0.344 m pitch and 0.045 m/s, so the
7/8-box fallbacks were not needed (their profiles exist and pass the layout tests but
were not run). Evidence is in `logs/orig_belt/`.

## Geometry (fixed belt, more boxes)

* Deck: y = -3.6 to +1.4 m (5.0 m), pick point near y = 0. A world test asserts the belt,
  frame and rollers of every generated world are identical to the three-box world.
* First box at y = -0.3 m; the last box stays 0.2 m inside the upstream end (y >= -3.4).
  So `pitch = floor(3.1 / (count - 1))`: **10 -> 0.344 m, 8 -> 0.442 m, 7 -> 0.516 m**
  (three-box: 1.2 m). Boxes are 80 x 50 x 50 mm, spawned square, 50 mm along the belt.
* Gripper clearance: the fingers close along the belt. A finger mesh reaches 26.4 mm past
  its joint, so fully open (0.04 m) its outer edge is 66.4 mm from the box centre. Allowing
  +-15 degrees of box yaw (34.5 mm half extent) and 35 mm of clear room gives
  `MIN_PITCH = 0.14 m`. Ten boxes leave 243 mm. The hand body bottom clears neighbouring box
  tops by only about 9 mm, but only matters within +-0.13 m of the target.
* Tote: unchanged from the earlier ten-box work (centre (0.16, -0.32), 0.54 x 0.24 m, ten
  separate floor slots, no stacking). 7 and 8 boxes use the first 7 and 8 slots.

## Capacity maths (why belt speed went DOWN, not up)

Box k+1 arrives every `a = pitch / belt speed`. The arm needs a cycle `T` (pick start to
back at the observation pose). Sustained operation needs `T <= a`; the early-launch window
(predicted y from -0.25 to -0.075, about 4 s of slack at 0.045 m/s) only absorbs a few late
cycles because lateness accumulates. For 10 boxes at 0.06 m/s, `a = 5.7 s`, but the original
cycle was 10.9 - 12.8 s. The belt speed that matches a cycle is `pitch / T`. A faster belt
therefore needs a faster arm, not the other way round. Total batch time is about
`10 x T`, whatever the belt speed, once the arm is the bottleneck.

## What shortened the cycle (7/8/10 profiles only)

| Change | Where | Effect |
|---|---|---|
| Gripper opens in 0.5 s (was 2.5 s) | `parcel_task.RELEASE_STROKE_S` -> `place(release_seconds)` | release phase 3.8 s -> about 1.0 s |
| No separate retract after release | `place(retract=False)`; the release point is 9 cm above the rim | one fewer stop |
| Slot confirmed from depth while returning | `tote_slot.SlotConfirmation`, `begin/settle_pending_slot` | about 0.7 s; next placement still waits for it |
| Persistent camera tracker | `parcel_continuous.enable_tracking` | no per-cycle rebuild (~0.5 s) |
| Early launch (predicted y >= -0.25 m, was -0.16) | `EARLY_LAUNCH_Y`, `BeltTracker.nearest(window)` | arm never waits for a box that is already whole and tracked |
| Shorter interception (lead 0.3 s, grasp 1.35 s, lift 0.9 s; was 0.5 / 1.6 / 1.3) | `Timing`, `FAST_TIMING` | intercept 4.5 s -> 3.65 s |
| Pre-dispatch infeasibility is retried | `InterceptNotFeasible` | an early box the arm cannot yet follow sends nothing and is retried closer |

Unchanged: grasp confirmation, hold and retention checks, collision checks, velocity /
acceleration audit limits (0.5 / 7.5), finger close dynamics, RRTConnect + TOTG baseline,
the whole three-box path (fixed 0.5 / 1.6 / 1.3 timeline, 2.5 s release, retract, blocking
slot confirmation), and the colour pipeline.

## Evidence (one source, 132 files, identical across runs)

All runs: harness `passed`, `task_exit` 0, `unexpected_errors` empty, and the independent
placed, geometry, before/after perception, continuous-transport and grasp-retention
checks all passed, with no tracebacks in `task.log`.

| Run | Mode | Placed | Slot error (mm) | Cycle PICK -> return (sim-s) min / mean / max | Mean arm idle | FULL at (sim / wall) |
|---|---|---|---|---|---|---|
| `p01` 0.04 m/s (before interception speed-up) | headless | 10/10 | 0.3 - 5.3 | 7.7 / 8.5 / 9.9 | 0.4 s | 101.0 s / 278 s |
| `p02` 0.045 m/s | headless | 10/10 | 2.1 - 5.3 | 6.4 / 7.2 / 8.7 | 0.7 s | 90.0 s / 306 s |
| `p03` 0.045 m/s | headless | 10/10 | 0.8 - 4.9 | 6.4 / 7.2 / 8.6 | 0.7 s | 92.2 s / 303 s |
| `gui01` 0.045 m/s | GUI | 10/10 | 0.2 - 5.3 | 6.2 / 7.0 / 8.4 | 0.9 s | 89.0 s / 469 s |

For comparison, the previous 10-box profile (0.9 m pitch on a belt lengthened by 5.2 m)
reached FULL at about 160 s and needed 10.9 - 12.8 s per box; this profile reaches it in
about 90 s on the original belt.

* The latest launch in any run was at y = -0.216 m; the tracker's fail-stop is at -0.075 m,
  so the closest approach was about 0.14 m (about 3 s) from failing. The slowest cycles are
  the far tote slots (`place` 2.3 s and return 1.7 s for slot 9 against 0.9 s and 0.8 s for
  slot 1). By a calculation from these measured cycles (NOT a run), a belt near 0.05 m/s
  would let the last boxes reach the fail-stop, so 0.045 m/s was kept; mean arm idle is
  about 0.7 s per box.
* Wall time depends on rendering and host load, not on the robot.
* Independent transport rule for 7/8/10: the earlier ">0.12 m of free travel" test assumed the
  box waits in the old window. With early launch it is replaced for these profiles by "observed
  at least 0.8 s, moving at >= 0.8 x commanded speed", plus the unchanged speed, following
  and prediction-error checks. The three-box rule is unchanged.

Regressions (default flags):

| Regression (same source) | Result |
|---|---|
| Three-box continuous, confirmed grasp, default flags, `reg_continuous3` | run1 PASS (3/3, all verdicts); **run2 FAILED**: `open_failed ... no result returned before timeout` at its first release. Real-time factor was 0.17 (host load average 10 - 14 on 12 cores; the editor's language server was at 105% CPU), against 0.33 - 0.48 in every passing run. The unchanged three-box path waits a fixed 15 wall-seconds for a 2.5 sim-s release. Evidence kept. |
| Three-box continuous rerun on a quieter host, `reg_continuous3_b` | run1 and run2 PASS (3/3, all six verdicts, RTF 0.48 / 0.41, no tracebacks) |
| Legacy colour sort, `reg_colour` | run1 and run2: 3/3 correct trays on ground truth, 0 task / planning / traceback / fatal counts; sorter exit 245 with one `Deleting MoveItCpp` each (the known legacy teardown exception, preserved, not fixed) |

So the three-box path passed 3 of 4 attempts. The one failure is an existing fixed-wall-timeout
weakness under heavy host load, not a change in behaviour. Applying the simulated-time gripper
deadline used by the 7/8/10 profiles to the three-box path would remove it; that was not done
because it changes the original path and was not requested.

## Limitations

* Only the 10-box pitch/speed above was run live. 7 and 8 boxes are accepted by the layout
  but not simulated.
* One arm: more boxes raise batch size, not service rate. The cycle is about 7.2 s on average
  and 8.7 s for the far slots.
* The ten-profile temporary camera uses a 76 degree HFOV (same pose and extrinsic, actual
  CameraInfo intrinsics), and a generated world with the wider tote and outrigger table.
* The three-box path waits a fixed 15 wall-seconds for the 2.5 sim-s gripper release. On a
  host that is heavily loaded (real-time factor below about 0.17) this can time out; it did
  once in the regression runs (below). The 7/8/10 profiles measure the wait in simulated time
  with a frozen-clock watchdog and a 120 wall-second ceiling, so they are not affected.
* Bringup logs still show harness-shutdown `KeyboardInterrupt` tracebacks in the detector and
  feed guard after `PARCEL_TASK_COMPLETE`, and a multiprocessing semaphore-leak warning
  persists. No hardware or fault-ladder certification is claimed.
* Failed or superseded evidence kept: `logs/ten_parcel/{pilot_01,gui_01,gui_02}` (earlier
  ten-box work) and `logs/orig_belt/reg_continuous3/run2` (host-load timeout).
