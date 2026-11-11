# Ten parcels from a moving conveyor: picking results

A simulated Franka Panda picks 10 cartons (80 × 50 × 50 mm, 50 g) from a 5.0 m conveyor while the belt keeps
moving, and places each one in its own slot of a single-layer tote. The camera never sees a simulator pose:
parcels are found in the calibrated overhead RGB-D depth image, tracked along the belt, and intercepted.
Every verdict below comes from an independent checker that reads Gazebo entity poses after the run, never from
the task's own log lines.

## Results (4 fresh runs, all 10/10)

| Run | Belt | Mode | Placed | Slot error (mm) | Cycle PICK → ready (sim-s) min / mean / max | Tote full at (sim-s / wall-s) |
|---|---|---|---|---|---|---|
| [`p01`](../../evidence/parcel_cell/p01/) | 0.040 m/s | headless | 10/10 | 0.3 – 5.3 | 7.7 / 8.5 / 9.9 | 101.0 / 278 |
| [`p02`](../../evidence/parcel_cell/p02/) | 0.045 m/s | headless | 10/10 | 2.1 – 5.3 | 6.4 / 7.2 / 8.7 | 90.0 / 306 |
| [`p03`](../../evidence/parcel_cell/p03/) | 0.045 m/s | headless | 10/10 | 0.8 – 4.9 | 6.4 / 7.2 / 8.6 | 92.2 / 303 |
| [`gui01`](../../evidence/parcel_cell/gui01/) | 0.045 m/s | GUI | 10/10 | 0.2 – 5.3 | 6.2 / 7.0 / 8.4 | 89.0 / 469 |

40 of 40 parcels placed upright, stable and inside their slot. In every run the harness passed, the task exited 0
with no unexpected errors, and the independent placement, geometry, before/after perception, continuous-transport
and grasp-retention checks passed. Slot error is the distance between the parcel centre and its slot centre.
Wall time depends on rendering and host load (these runs reached 0.2–0.36× real time on the test machine).

`p01` ran before the interception speed-up below; `p02`, `p03` and `gui01` use the final settings.

## How it works

1. **Perception.** `parcel_perception.py` segments the top plane of each carton in the depth image, independent of
   carton or tote colour, and deprojects it through the hand-eye-calibrated camera transform
   (`config/overhead_camera_extrinsic.yaml`, Kabsch SVD fit, RMS residual 0.37 mm).
2. **Tracking.** `parcel_continuous.py` keeps one belt tracker alive for the whole run and predicts where each whole,
   tracked carton will be.
3. **Interception.** As soon as a carton is reachable (predicted y ≥ −0.25 m) the arm launches a collision-checked
   intercept with MoveIt 2: 0.3 s planning lead, grasp at 1.35 s, 0.9 s lift. If the arm cannot yet follow, nothing is
   sent and it retries as the carton comes closer.
4. **Confirmed grasp.** Two finger-pad contact sensors must both report contact before the lift is accepted
   (`parcel_confirmed.py`).
5. **Place.** Gripper opens in 0.5 s, no separate retract; the slot is confirmed from depth while the arm already
   heads back (`tote_slot.py`).

Pitch is derived from the count so the belt length never changes: 10 boxes at 0.344 m pitch, belt 0.045 m/s,
7.6 simulated seconds between arrivals.

## Safety behaviour

Lost perception or command messages, stale camera frames, out-of-region or invalid detections, a failed intercept or
a full tote stop the feed. A carton the arm can no longer reach is never chased: the tracker's fail-stop is at
y = −0.075 m, and the latest launch in any run above was at y = −0.216 m (about 3 s of margin).

## Reproduce

```bash
# headless, scored, one fresh run (writes its own verdict files)
env -i PATH=/usr/bin:/bin:/usr/local/bin bash --noprofile --norc \
  tools/run_parcel_cell_check.sh --mode continuous --grasp-mode confirmed \
  --parcel-count 10 --belt-speed 0.045 --runs 1 --logdir /tmp/parcel-check
```

## Limits

Simulation only. Known-size aligned cartons. The tote holds 10 slots and the batch ends when it is full. Raising the
belt to about 0.05 m/s would, by a calculation from the measured cycles (not a run), let the last boxes reach the
fail-stop, so 0.045 m/s is the validated speed.
