# Parcel task waiting at ready — GUI investigation

The user's two saved task logs reached `READY` and `FEED`, but contained no `PICK` event.
The original failure text was printed only to Terminal 2 and was not retained in the ROS
log. The simulator was already stopped by the user when inspected. The exact original
cause therefore remains unconfirmed; two invocations were recorded, but overlap or an
indexed/stationary mode mismatch cannot be proved from those logs alone.

## New observations

- Real GUI conveyor probe passed: all three cartons moved +17.185 mm, stopped with no
  measured drift after command loss, and invalid speeds left the conveyor stopped.
- Two complete GUI runs of the existing motion code passed all three placements each.
  Evidence: `logs/parcel_gui_debug/indexed_gui_before/`. The reported failure did not
  reproduce in that clean, single-task GUI setup.
- The first delivery takes approximately 20–30 wall seconds in this GUI environment.
  During that interval the arm is expected to remain at ready while the belt moves.
- User log extracts: `logs/parcel_gui_debug/user_observed_events.json`. Original logs:
  `~/.ros/log/python3_469484_1794190179698.log` and
  `~/.ros/log/python3_469880_1794190222868.log`.

## Changes

- `parcel_task.py`: print `FEED_WAIT` progress every five wall seconds, including current
  simulation time, detected count, command speed, guard subscriber count and data ages.
  Task failures also go to the ROS logger. Multiple-target errors explain the paired
  launch/task flags. No planning, speed, perception freshness or stop thresholds changed.
- `parcel_task_session.py`: one process per user/ROS domain can run `parcel_task`; a second
  is rejected before controller setup. Kernel file locks release on process exit, including
  a crash, so an old PID file cannot permanently block retries.
- `tools/run_parcel_cell_check.py`: optional `--gui` exercises the actual GUI launch and
  records that mode in the verdict. Existing headless mode remains the default.
- `tools/test_parcel_cell.py`: tests overlapping domains, successful release/reacquisition,
  and release after abrupt process exit. Expanded offline suite passes (seven parcel tests).

## Verification

- Live duplicate test: updated task PID 480720 held the domain-86 lock. A second executable
  returned exit 1 with `already running`, no task events, and no controller initialization.
  The original run continued. Evidence: `logs/parcel_gui_debug/duplicate_task_verified.json`.
- Updated GUI validation passed both fresh runs: 6/6 actual cartons upright, stable and
  fully inside the tote. Geometry, transport, ROS perception, camera evidence and operational
  log health passed in both. Evidence: `logs/parcel_gui_debug/indexed_gui_after/`.
  Both source manifests match the current implementation. Both completed task locks were
  successfully reacquired, confirming retries are possible after the known teardown exit.
- A first artificial duplicate probe was attempted before the old task loaded the new lock;
  it failed ROS logging initialization in a minimal sandbox environment and moved no arm.
  That probe is not counted as duplicate-guard evidence.

## Test it

From a desktop terminal in `~/ros_test_ws_mirror`:

```bash
env -i PATH=/usr/bin:/bin:/usr/local/bin \
  DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash --noprofile --norc tools/run_parcel_cell_check.sh \
  --gui --mode indexed --runs 1 --domain 86 \
  --logdir "logs/gui_check_$(date +%Y%m%d_%H%M%S)"
```

For the two-terminal workflow, source the same ROS base and mirror overlay and use the same
ROS domain in both. Launch with `indexed:=true` and run `parcel_task --indexed` once. Let
`FEED` finish; do not start another task while waiting. Restart the scene for a fresh batch
after the three tote slots are filled. Read any `PARCEL_TASK_FAILED` reason before retrying;
exit 245 alone is not proof of success. The known MoveIt teardown exception is accepted only
after task completion and passing independent placement/perception checks.

If `FEED_WAIT` simulation time stays fixed, check that Gazebo is playing. The original
failure cause remains unconfirmed without its Terminal 2 failure text; no simulator or
motion failure was reproduced in four fresh GUI batches (two before and two after these
diagnostic/startup changes). All changes are local to the mirror and uncommitted.
