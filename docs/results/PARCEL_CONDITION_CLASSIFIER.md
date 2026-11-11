# Parcel condition classifier: damaged vs intact

Held-out test: 30/30 parcels correct, damaged recall 232/232 crops. Routing the arm by the classifier:
3 independent 10-box runs, 30/30 parcels in the right place. Simulation only: sim accuracy is NOT real-world accuracy.

Every number below comes from a command whose output is quoted or from a file in
[`evidence/parcel_condition/`](../../evidence/parcel_condition/). Paths written as `logs/condition/...` are the local
working copies: their small files (manifests, splits, metrics, run verdicts) are in `evidence/parcel_condition/`,
the image crops are the release asset `condition-dataset-r1.zip`, the model is `condition-model-r1.onnx`, and the
7.2 GB of rosbags are not published.

## Step 1 - problem study and decision

| Candidate | What depth already solves | What only RGB sees | Label truth in Gazebo |
|---|---|---|---|
| Size classes (S/M/L) | Box dimensions from the depth top-plane blob (`parcel_perception.py`) | nothing extra | known from the spawned model |
| Damaged vs intact | Position and footprint only; the top plane is flat to < 1 mm for both | tears, stains, crushed corners (surface appearance) | known by construction: the spawner decides which parcels get the damage visuals |
| Both | as above | as above | more classes, more data, no new insight |

**Decision: damaged vs intact**, classes `intact` / `damaged`. Depth gives geometry for free, so a network adds
nothing for size. Condition is a pure appearance property that depth cannot see, so it is the one place a
learned model can add information.

Architecture decision: the existing depth detector localises the parcel; a small image
classifier labels the RGB crop. No box-detector network is trained.

## Step 2 - RGB path

```
ros2 topic info /overhead_camera/image  -> sensor_msgs/msg/Image, 1 publisher
ros2 topic hz   /overhead_camera/image  -> average rate: 4.994 (wall clock, headless sim)
frame: rgb8 640x480, parcels about 30 px wide, sharp, belt runs left-right in the image
```

- Segmentation camera: NOT added. The camera lives in the shared Panda URDF (`overhead_rgbd`), and the
  calibrated detector depends on it. Labels instead come from Gazebo entity poses projected through the
  calibrated camera (`overhead_camera_extrinsic.yaml` + `camera_info`). The projection is verified by drawing
  boxes on frames (step 5).
- Consequence for step 3: damage marks must be large enough to remain visible at about 30 px per parcel.

## Step 3 - damaged parcels in the simulator

`parcel_variants.py` writes one SDF per parcel from `models/parcel_box.sdf`. Collision box, mass and inertia are
copied untouched; only `<visual>` elements change (carton shade, label/barcode position, optional benign sticker
on BOTH classes, and for damaged parcels 1-2 of: stain, tear with flap, crushed corner with creases). Every decal
lies inside the top face and rises at most 0.8 mm above it (the depth detector's plane tolerance is 6 mm).
Launch: `variant_seed:=<s> variant_damaged:=2,5,9 variant_manifest:=<json> variant_yaw_deg:=10`
(default OFF: with no `variant_seed` every parcel is the original `parcel_box.sdf`).
Offline test `tools/test_parcel_variants.py` (physics signature identical for 60 variants, decals bounded,
deterministic, all three damage kinds occur) is part of `tools/run_offline_tests.sh`.

**Damage marks were enlarged ~1.4x after the first look** (stain radius 10-17 mm, tear 32-46 mm, crush 16-22 mm):
at about 30 px per parcel the first version was only 4-6 px. The marks are therefore large and obvious.

Arm picks the variants unchanged (full 10-box task, `tools/condition_arm_check.sh`, independent
`tools/check_parcel_cell.py --placed --parcel-count 10`):

| run | damaged parcels | task | checker | slot errors (mm) |
|---|---|---|---|---|
| armA | 2,5,6,9 | PARCEL_TASK_COMPLETE, FULL at 87.6 sim-s | passed, exact entities | 1.0 - 5.3 |
| armB | 1,3,7,8,10 | PARCEL_TASK_COMPLETE | passed | 1.0 - 5.3 |
| armC | 2,3,8 | PARCEL_TASK_COMPLETE | passed | 1.2 - 6.2 |

0 `Traceback` in each task log. The `rcutils_reset_error` lines in the log also appear in the earlier baseline
runs (see [`evidence/parcel_cell/`](../../evidence/parcel_cell/)), so they are not new.

**Depth detector is blind to the condition** (all 16 recordings, parcels inside the detector window):

| class | in window | detected | rate | mean xy error | median |
|---|---|---|---|---|---|
| intact | 1229 | 1130 | 91.9 % | 1.19 mm | 0.90 mm |
| damaged | 1224 | 1113 | 90.9 % | 1.32 mm | 1.00 mm |

Detection rate difference p = 0.41 (chi-square); position error difference p = 0.09 (Mann-Whitney). Neither is
significant at 0.05; the 0.13 mm mean gap is a fraction of a pixel. The depth detector has no class output at all.

## Step 4 - dataset recording

`tools/condition_collect.sh plan 16` then `run`: one fresh headless sim per recording (10 boxes, original belt,
belt 0.07 or 0.08 m/s, spawn yaw jitter +-10 deg, 3-7 of 10 parcels damaged in a random order, per-recording
seed). `tools/condition_run.py` republishes Gazebo parcel poses at 20 sim-Hz as simulation-only ground truth and
records an MCAP bag with `ros2 bag record --use-sim-time`:
`/overhead_camera/image`, `/overhead_camera/camera_info`, `/clock`, `/tf`, `/tf_static`, `/joint_states`,
`/gt/parcel_poses`, `/parcel_cell/moving_detections`, `/parcel_cell/detections`, `/parcel_cell/belt_speed`.

Deviations from the plan, with reasons:
- **Raw depth is not in the bag** (it halved the I/O at a sim real-time factor of about 0.2); the live depth
  detector's own output (`/parcel_cell/moving_detections`) is recorded instead, which is what step 7 compares.
- **No Gazebo segmentation camera**; labels come from the spawn-time manifest plus projected Gazebo poses
  (step 2). `ros_gz_bridge` drops entity names for `Pose_V`, so `condition_run.py` subscribes through
  `gz.transport13` directly.
- Recording ends when the last parcel passes y = 0.15 m: the belt queues parcels against an end stop near
  y = 0.74, and touching parcels are not a belt view (the extractor also drops any parcel within 0.12 m of another).

Result: 16 of 16 recordings ok, none fell, 44-51 sim-s each (about 3 min wall), 7.2 GB of bags in `logs/condition/rec/`.
Split by whole recording (`logs/condition/splits.json`): train r01 r02 r03 r05 r06 r07 r08 r12 r13 r15 r16 (11),
val r04 r10, test r09 r11 r14.

## Step 5 - crops and label check

`tools/condition_extract.py` samples a frame every 0.5 sim-s and cuts, per parcel, a fixed 0.12 m world window
(`parcel_condition.py`, 64x64 px) at (a) the Gazebo pose and (b) the matching real depth-detector box.
Dataset manifest: `logs/condition/dataset/dataset_manifest.json` (git commit a9b3ad6, crop parameters, per-recording counts).

| split | intact crops | damaged crops | depth-detector crops |
|---|---|---|---|
| train | 1578 | 1478 | - |
| val | 297 | 299 | - |
| test | 319 | 464 | 402 |
| total | 2194 | 2241 (4435) | 2243 |

Label checks by eye: projected boxes drawn on frames (`logs/condition/dataset/check/`, boxes hug the parcels, detector
crosses land on parcels) and contact sheets of 20 random crops per class from both crop sources
(`tools/condition_sheet.py`): damaged crops show a stain, tear or crushed corner, intact crops do not, crops centred.

Licence record: code is torchvision (BSD-3-Clause). The weights are torchvision `ResNet18_Weights.IMAGENET1K_V1`,
pre-trained on ImageNet. ImageNet's terms limit the dataset to non-commercial research and education, and whether
that carries to derived weights is unsettled: fine for this research project, check before any commercial use.
ONNX Runtime is MIT, PyTorch is BSD-style. No AGPL (YOLO) component is used.

## Step 6 - training

`tools/condition_train.py` in `~/venvs/parcel_cls` (torch 2.15.0+cu132, GTX 1650): ResNet18 pretrained, 2 outputs,
128 px input, batch 64, AMP, 3 epochs head only (lr 1e-3) then 20 epochs full (AdamW lr 1e-4, cosine), brightness /
contrast / noise / small affine augmentation, class-weighted loss, TensorBoard in `runs/r1/tb`. Gazebo was stopped.
Best epoch by validation balanced accuracy: epoch 8 (0.993). Whole run about 7 min, peak VRAM 553 MiB.
Train loss reaches 0 while val loss settles near 0.05, so the model overfits the training parcels somewhat.
Artifacts: `logs/condition/runs/r1/{best.pt,model.onnx,config.json,history.json,eval_test.json,eval_val.json}`.

## Step 7 - held-out evaluation (test = r09, r11, r14; 30 parcels, 17 damaged; model.onnx on CPU)

| input | level | n | accuracy | damaged precision | damaged recall | intact recall | confusion (intact as damaged / damaged as intact) |
|---|---|---|---|---|---|---|---|
| Gazebo-pose crops | per crop | 783 | 0.990 | 0.983 | 1.000 | 0.975 | 8 / 0 |
| Gazebo-pose crops | per parcel | 30 | 1.000 | 1.000 | 1.000 | 1.000 | 0 / 0 |
| depth-detector crops | per crop | 402 | 0.993 | 0.987 | 1.000 | 0.982 | 3 / 0 |
| depth-detector crops | per parcel | 30 | 1.000 | 1.000 | 1.000 | 1.000 | 0 / 0 |

Per parcel = mean damaged probability over that parcel's frames (13.4 frames per parcel in the real pipeline).
Threshold sweep on depth-detector crops, damaged recall / intact-sent-to-reject: 0.3: 1.000 / 4.1 %,
0.5: 1.000 / 1.8 %, 0.7: 1.000 / 0 %, 0.9: 1.000 / 0 %. Validation (r04, r10): 0.993 per crop, 20/20 parcels.

Comparison with the depth detector on the same frames: it finds the parcels (91-94 % of those inside its window,
1.0-1.4 mm position error) but cannot separate the classes, so the best class-blind answer is one fixed label:
always-intact gives damaged recall 0.00, always-damaged gives recall 1.00 but sends 100 % of intact parcels to reject.

**How far to trust this.** 30 test parcels is small: the 95 % lower bound for 17/17 damaged recall is about 0.80
and the upper bound for 0/13 false rejects about 0.25 (per parcel); per crop, 232/232 gives a bound near 0.98 and
3/170 false rejects is 1.8 %. All recordings share one camera, one belt, one lighting and one damage generator
(hand-made decals), and the marks are large. This shows the pipeline works and that RGB sees what depth cannot.
It does not show real-world accuracy.

## Step 8 - ONNX challenger node (log only)

`parcel_condition_node.py` (run with `tools/run_condition_node.sh <model.onnx>`: training venv for onnxruntime plus
this checkout's ROS overlay) subscribes to the RGB image, camera_info and the depth detector's
`/parcel_cell/moving_detections`, cuts the same crop, runs the ONNX model on CPU and publishes
`/parcel_cell/condition` (Detection3DArray). The arm does not read it. Live, during a full 10-box arm task
(`tools/condition_arm_check.sh armC 2,3,8 <model>`, node started before the task, checked against Gazebo by
`tools/condition_live_check.py`):

- 249 classified detections: damaged->damaged 74, intact->intact 174, intact->unknown 1 (below 0.75 confidence),
  0 wrong; 10 of 10 parcels correct by majority; mean confidence 0.993.
- Latency per detection message: mean 15 ms, p95 about 29 ms, worst 141 ms (once). At 0.045 m/s that is under
  1.3 mm of belt travel (6 mm worst case).
- 56 of about 305 detection messages (18 %) had no RGB frame within 60 ms and were skipped, still about 25 classified
  frames per parcel.
- The arm task passed the independent checker with the node running (armB 10/10, armC 10/10).

## Step 9 - let the classifier drive the arm (behind flags, default OFF)

**Pass condition.** Two thresholds gate the step from logging to routing: damaged recall >= 0.95 and
intact-sent-to-reject <= 0.05, per parcel on the held-out recordings. Measured (step 7,
real depth-detector crops): damaged recall 17/17 and false rejects 0/13 per parcel. Both meet the proposal on the
point estimates, but 30 parcels is small (95 % bounds: recall >= 0.80, false-reject rate <= 0.25), so this clears the
gate to *try* it in the simulator, not a guarantee. `parcel_reject.py` keeps the vote thresholds as named constants.

**What changed (every change is opt-in; offline suite and a default run prove the old path is intact).**
a) Reject bin: `reject_bin:=true` (launch) / `--condition-routing` (task) adds `reject_bin_red` + `reject_station` to the
   world (`parcel_batch_world.py`, via `parcel_reject.add_reject_bin`): 0.34 x 0.24 m interior like the tote, floor at
   the tote floor height, centred at (0.18, 0.50) on the downstream side, clear of the pedestal, belt rail and camera
   fixture (unit test: no overlap with any other solid). Six drop slots at the tote's 0.10 m pitch. The default world is
   byte-identical for 3/7/8/10 parcels (sha256 compared against the committed generator). The bin is a static solid in
   the MoveIt planning scene automatically, and each rejected parcel is added as a collision object like a filled slot.
b) Arm logic (`parcel_task.py`, `parcel_continuous.py`, `parcel_tracking.py`, `parcel_io.py`): per-frame classifier
   output from `/parcel_cell/condition` is attached to the nearest belt track as a vote. The mean damaged probability
   over the latest 8 votes (at least 4) decides: >= 0.75 damaged -> reject bin, <= 0.25 intact -> next tote slot,
   anything between, or too few votes by y = -0.12 m -> `SKIPPED`: the parcel is excluded from the tracker's window
   check, left on the belt and logged, never guessed. A full bin or tote raises.
c) Fallback: no `--condition-routing`, no `reject_bin:=true`, no node = the original depth-only behaviour, one switch.
d) Independent check: `tools/check_condition_routing.py` reads Gazebo poses and the spawn-time truth (never the task's
   status messages): damaged must rest stable and upright inside the bin, intact in a distinct tote slot within 15 mm.

**Evidence** (`tools/condition_arm_check.sh` with `ROUTING=1`, headless, domain 86; 0 Tracebacks in every task log):

| run | what | damaged | result | FULL at |
|---|---|---|---|---|
| routeA | routing, trained model | 2,5,7 | independent check 10/10: 3 in bin, 7 in tote, no misroute | 88.4 sim-s |
| routeB | routing, different seed | 1,3,4,8,10 | 10/10: 5 in bin, 5 in tote | 88.4 sim-s |
| demo | `parcel_demo.sh` exactly as in the README | 3,6,9 | 10/10: 3 in bin, 7 in tote | 90.9 sim-s |
| skipA | routing, deliberately uncertain model (final layer zeroed, p = 0.5) | 2,5,7 | all 10 SKIPPED (verdict unknown, 7-24 votes each), nothing picked, checker reports all 10 left on the belt, no exception | 81.0 sim-s |
| default | stock `parcel_demo.sh`, no variants, no routing | - | `check_parcel_cell.py` 10/10 in the tote, slot errors 1.7-6.4 mm | 91.6 sim-s |

Per-frame classifications during routeA/routeB still contained a few errors (e.g. routeA 6 of 315 intact crops voted
damaged), and the 8-vote mean fixed all of them: 30 of 30 parcels were routed correctly across routeA, routeB and the
demo run. Routing did not slow the cell (88-91 sim-s against 91.6 for the default). The skip run proves the
unknown path end to end; it is a deliberate fault injection, not something the trained model does.

**Limits.** The reject bin holds 6 parcels (a seventh damaged parcel stops the task with "Reject bin is full"). Skipped
parcels ride the belt to the end stop and are not retried. The classifier was trained and tested on simulated parcels
with large decal damage under one camera and lighting: this demonstrates the pipeline and the control logic, not
real-world damage detection. Next real step if wanted: harder / subtler damage, more lighting and belt variation, and
more held-out recordings before trusting a threshold.
