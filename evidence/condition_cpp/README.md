# C++ condition node: parity and latency evidence (Phase B2, 2026-11-13)

The C++ port (`src/twin_condition_cpp`) was run next to the Python node (`parcel_condition_node.py`) on three recorded
test bags (r09, r11, r14; not used for training). Same model (`model.onnx`, sha256 f94d9d35…892405), same inputs.

| Bag | Detections classified by both | Max abs prob diff | Mean abs prob diff | Decision mismatches |
|---|---|---|---|---|
| r09 | 333 | 8.4e-8 | 2.7e-8 | 0 |
| r11 | 347 | 8.4e-8 | 2.8e-8 | 0 |
| r14 | 347 | 8.4e-8 | 3.0e-8 | 0 |

Pass rule: 0 decision mismatches and max prob diff < 1e-3. All three pass (`parity_r*.json`).

Coverage caveat: 10-17 detections per bag were classified by only one node. Each node pairs a detection with an image
within 0.06 s; under replay with best-effort QoS some frames arrive too late for one node, so that node logs the
detection as unmatched. This is timing, not a numerical difference, and the Python node shows it too. It is mostly CPU
contention: with both nodes running about 55% of detections are unmatched, with one node alone about 10% (52/499
Python, 55/568 C++).

Latency (one node alone on r09, `latency_lines.txt`): Python mean 12.8 ms / p95 17.5 ms, C++ mean 12.5 ms / p95 18.6 ms.
They are about equal because ONNX Runtime inference dominates; the port's value is a deployable C++ node with
proven parity, not speed. With both nodes running at once, each is slower (shared CPU).

Reproduce (no Gazebo; own ROS domain 78):

    tools/run_condition_parity.sh <bag_dir> logs/condition_cpp/parity/r09 both
    python3 tools/compare_condition_bags.py logs/condition_cpp/parity/r09/out
    tools/run_condition_parity.sh <bag_dir> logs/condition_cpp/parity/alone_cpp cpp
