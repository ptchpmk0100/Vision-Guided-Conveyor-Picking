# Parcel condition classifier: training evidence

Everything needed to inspect how the damaged/intact classifier was made. All data is simulated (Gazebo).
The narrative write-up is `../../PARCEL_CONDITION_CLASSIFIER.md`. Code is in `src/twin_moveit_scripts/` and `tools/condition_*`.

## What is here

| Path | Content |
|---|---|
| `plan.json`, `splits.json` | The 16 planned recordings: seed, damaged parcel numbers, belt speed, and the train / val / test split |
| `recordings/rNN/` | Per recording: `manifest.json` (spawn-time label of every parcel) and `run_summary.json` |
| `dataset/index.csv` | One row per crop: recording, frame, parcel, label, true x/y/z/yaw, crop path, depth-detector crop path and error |
| `dataset/crops/` | 4435 ground-truth crops (`*_gt.png`) and the crops cut at the depth detector's boxes (`*_det.png`), sorted by recording and class |
| `dataset/check/` | Full camera frames with labels, crop windows and detections drawn on, used to check the labels by eye |
| `dataset/dataset_manifest.json` | Git commit, crop window (0.12 m), crop size (64 px), class counts per recording |
| `training/r1/config.json` | Every training hyper-parameter and the data sizes |
| `training/r1/history.json`, `training/train_r1.log`, `training/r1/tb/` | Per-epoch loss and validation metrics, console log, TensorBoard events |
| `training/r1/eval_val.json`, `eval_test.json` | Validation and held-out test results, per crop and per parcel, plus the depth-detector-only comparison |
| `arm_runs/` | Task results and independent Gazebo checker output for the arm runs (routeA, routeB, skipA, armA to armC) |
| `figures/` | Contact sheets, a camera strip and `training_curves.png` (made by `make_curves.py`) |

Not included (too large for git): the 16 rosbags (7.2 GB) and the model weights (`best.pt`, `model.onnx`, 43 MB each).
Regenerate them with the commands below, or publish the weights as a GitHub Release asset.

## Pipeline

1. `tools/condition_collect.sh plan 16` then `run`: one fresh headless simulation per recording, parcels spawned as intact or damaged variants (`parcel_variants.py`), RGB + camera info + depth detections + Gazebo poses recorded to MCAP.
2. `tools/condition_extract.py --recordings logs/condition/rec --out logs/condition/dataset --gap 0.5`: crops and `index.csv`.
3. `tools/condition_train.py --dataset ... --splits splits.json --out ...`: ResNet18 fine-tuning, exports ONNX.
4. `tools/condition_eval.py ... --split test`: held-out evaluation.
5. Live: `tools/parcel_demo.sh sim|task` with `CONDITION_MODEL=...` (see the main README).

## Parameters (from `training/r1/config.json`)

ResNet18 (torchvision ImageNet weights), 128 px input, batch 64, 3 head-only epochs (lr 1e-3) then 20 full epochs
(AdamW 1e-4, cosine, weight decay 1e-4), class-weighted loss, mixed precision, seed 0. Augmentation: brightness and
contrast ±15%, noise, rotation ±8°, scale ±6%, small shift. Best epoch chosen on validation balanced accuracy (epoch 8).
Split by whole recording: 11 train (3056 crops), 2 val (596), 3 test (783).

## Results

Held-out test, crops cut at the real depth detector's boxes, 30 parcels: damaged recall 232/232 crops, 3 of 170 intact
crops flagged, 30/30 parcels correct after the 8-frame vote. Arm routing: three 10-box runs, 30/30 parcels in the right
place, checked independently against Gazebo. See `training/r1/eval_test.json` and `arm_runs/`.

## Limits

Simulation only, large synthetic damage marks, one camera and lighting setup, 30 test parcels (95% lower bound of damaged
recall about 0.80). Training loss reaches 0 while validation loss stays near 0.05, so the model overfits its training
parcels somewhat. ImageNet pretrained weights carry research-use terms; check before commercial use.
Some logs contain local file paths from the machine that produced them.
