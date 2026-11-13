#!/usr/bin/env python3
"""Compare the Python and C++ condition nodes on one recorded replay (B2 parity check).

Reads a bag that holds BOTH outputs (/parcel_cell/condition from the Python node, /parcel_cell/condition_cpp from
the C++ node) and matches detections by (header stamp, detection id). Prints one JSON object:

  n_both, n_python_only, n_cpp_only        detections found on both / only one side
  max_abs_prob_diff, mean_abs_prob_diff    over the two class probabilities (results[1], results[2])
  decision_mismatches                      label or 'unknown' differs (results[0].class_id)
  worst                                    the 3 largest differences

Exit code 0 when decision_mismatches == 0 and max_abs_prob_diff < --tol (default 1e-3) and n_both > 0.

    source /opt/ros/jazzy/setup.bash
    python3 tools/compare_condition_bags.py out_r09/out
"""
import argparse
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PY_TOPIC, CPP_TOPIC = '/parcel_cell/condition', '/parcel_cell/condition_cpp'


def read(uri):
    meta = rosbag2_py.Info().read_metadata(uri, '')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=meta.storage_identifier),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {PY_TOPIC: {}, CPP_TOPIC: {}}
    msgs = {PY_TOPIC: 0, CPP_TOPIC: 0}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in out:
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        msgs[topic] += 1
        st = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        for det in msg.detections:
            r = det.results
            if len(r) < 3:
                continue
            out[topic].setdefault((st, det.id), dict(
                label=r[0].hypothesis.class_id, conf=r[0].hypothesis.score,
                probs=(r[1].hypothesis.score, r[2].hypothesis.score)))
    return out, msgs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--tol', type=float, default=1e-3)
    args = ap.parse_args()
    out, msgs = read(args.bag)
    py, cpp = out[PY_TOPIC], out[CPP_TOPIC]
    both = sorted(set(py) & set(cpp))
    diffs, mismatches = [], []
    for key in both:
        d = max(abs(a - b) for a, b in zip(py[key]['probs'], cpp[key]['probs']))
        diffs.append((d, key))
        if py[key]['label'] != cpp[key]['label']:
            mismatches.append(key)
    worst = [dict(stamp=f'{k[0][0]}.{k[0][1]:09d}', id=k[1], diff=d, python=py[k], cpp=cpp[k])
             for d, k in sorted(diffs, reverse=True)[:3]]
    result = dict(
        bag=args.bag, python_messages=msgs[PY_TOPIC], cpp_messages=msgs[CPP_TOPIC],
        n_both=len(both), n_python_only=len(set(py) - set(cpp)), n_cpp_only=len(set(cpp) - set(py)),
        max_abs_prob_diff=max((d for d, _ in diffs), default=None),
        mean_abs_prob_diff=(sum(d for d, _ in diffs) / len(diffs)) if diffs else None,
        decision_mismatches=len(mismatches), tolerance=args.tol, worst=worst)
    ok = bool(both) and not mismatches and result['max_abs_prob_diff'] < args.tol
    result['passed'] = ok
    print(json.dumps(result, indent=1, default=str))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
