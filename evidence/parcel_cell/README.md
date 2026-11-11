# Ten-parcel picking: run evidence

Verdict files from four fresh 10-parcel runs on the moving conveyor, summarised in
[`docs/results/PARCEL_PICKING.md`](../../docs/results/PARCEL_PICKING.md). Produced by
`tools/run_parcel_cell_check.sh --mode continuous --grasp-mode confirmed --parcel-count 10`.

| File | Content |
|---|---|
| `summary.json` | Harness verdict: passed, task exit code, unexpected errors, settings |
| `placed.json` | Independent Gazebo check of every parcel: slot, position, slot error, upright, stable, inside footprint |
| `geometry.json` | Scene geometry check before the run |
| `detections_before.json`, `detections_after.json`, `perception_snapshot.json` | Camera detections compared with Gazebo poses |
| `continuous_verdict.json` | Belt transport check: speed, following, prediction error |
| `grasp_verdict.json` | Grasp retention per parcel (two-pad contact, hold, lift) |
| `task.json` | Task event timeline (PICK, INTERCEPT, GRASP_CONFIRMED, PLACE, FULL) with simulation and wall time |
| `result.json`, `source_manifest.json` | Run result and hashes of the source files that ran |

Not included: the per-run belt transport trace (`transport.json`, 1.3 MB each), camera images and raw ROS logs.
