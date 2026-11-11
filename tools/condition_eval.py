#!/usr/bin/env python3
"""Evaluate the exported ONNX condition classifier on held-out recordings and compare with the depth detector.

    ~/venvs/parcel_cls/bin/python tools/condition_eval.py --dataset logs/condition/dataset \
        --splits logs/condition/splits.json --model logs/condition/runs/r1/model.onnx --split test

Only recordings from the chosen split are read. Reports (a) ground-truth crops, (b) crops cut at the REAL
depth detector's boxes, each per crop and per parcel (mean damaged probability over a parcel's frames).
"""
import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.parcel_condition import preprocess, softmax  # noqa: E402  (pure numpy module)


def metrics(y, p):
    y, p = np.asarray(y), np.asarray(p)
    tp, fp = int(((p == 1) & (y == 1)).sum()), int(((p == 1) & (y == 0)).sum())
    fn, tn = int(((p == 0) & (y == 1)).sum()), int(((p == 0) & (y == 0)).sum())
    r = lambda a, b: a / b if b else float('nan')  # noqa: E731
    return dict(n=len(y), confusion={'intact_as_intact': tn, 'intact_as_damaged': fp,
                                     'damaged_as_intact': fn, 'damaged_as_damaged': tp},
                damaged_precision=r(tp, tp + fp), damaged_recall=r(tp, tp + fn),
                intact_precision=r(tn, tn + fn), intact_recall=r(tn, tn + fp), accuracy=r(tp + tn, len(y)),
                intact_sent_to_reject_rate=r(fp, fp + tn))


def classify(session, paths, root):
    crops = [cv2.cvtColor(cv2.imread(str(root / p)), cv2.COLOR_BGR2RGB) for p in paths]
    probs = []
    for i in range(0, len(crops), 256):
        out = session.run(None, {'input': preprocess(crops[i:i + 256])})[0]
        probs.append(softmax(out)[:, 1])
    return np.concatenate(probs) if probs else np.array([])


def sweep(y, prob):
    rows = []
    for t in (0.3, 0.5, 0.7, 0.9):
        m = metrics(y, (prob >= t).astype(int))
        rows.append(dict(threshold=t, damaged_recall=m['damaged_recall'], intact_sent_to_reject_rate=m['intact_sent_to_reject_rate']))
    return rows


def evaluate(session, rows, column, root):
    rows = [r for r in rows if r[column]]
    if not rows:
        return None
    y = np.array([1 if r['label'] == 'damaged' else 0 for r in rows])
    prob = classify(session, [r[column] for r in rows], root)
    per_parcel = defaultdict(list)
    for r, pr in zip(rows, prob):
        per_parcel[(r['recording'], r['parcel'], r['label'])].append(pr)
    yp = np.array([1 if k[2] == 'damaged' else 0 for k in per_parcel])
    pp = np.array([np.mean(v) for v in per_parcel.values()])
    return {'per_crop': metrics(y, (prob >= 0.5).astype(int)), 'per_crop_sweep': sweep(y, prob),
            'per_parcel': metrics(yp, (pp >= 0.5).astype(int)), 'parcels': len(per_parcel),
            'frames_per_parcel_mean': float(np.mean([len(v) for v in per_parcel.values()]))}


def depth_report(rows):
    """What the depth detector gives per class: it finds parcels; it has no class output."""
    out = {}
    for cls in ('intact', 'damaged'):
        sub = [r for r in rows if r['label'] == cls]
        in_window = [r for r in sub if .505 <= float(r['x']) <= .615 and -.4 <= float(r['y']) <= .18]
        found = [r for r in in_window if r['det_found'] == '1']
        err = [float(r['det_err_mm']) for r in found]
        out[cls] = dict(gt_crops=len(sub), in_detector_window=len(in_window), detected=len(found),
                        detection_rate=len(found) / len(in_window) if in_window else float('nan'),
                        mean_err_mm=float(np.mean(err)) if err else float('nan'),
                        std_err_mm=float(np.std(err)) if err else float('nan'))
    try:
        from scipy.stats import mannwhitneyu
        a = [float(r['det_err_mm']) for r in rows if r['label'] == 'intact' and r['det_found'] == '1']
        b = [float(r['det_err_mm']) for r in rows if r['label'] == 'damaged' and r['det_found'] == '1']
        out['position_error_mann_whitney_p'] = float(mannwhitneyu(a, b).pvalue) if a and b else None
    except Exception:  # noqa: BLE001
        out['position_error_mann_whitney_p'] = None
    # class-blind: no class output, so the best it can do is one fixed answer
    n_d = sum(r['label'] == 'damaged' for r in rows)
    out['class_blind_baseline'] = dict(
        always_intact=dict(damaged_recall=0.0, accuracy=1 - n_d / max(1, len(rows))),
        always_damaged=dict(damaged_recall=1.0, intact_sent_to_reject_rate=1.0, accuracy=n_d / max(1, len(rows))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--splits', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--split', default='test', choices=['train', 'val', 'test'])
    ap.add_argument('--out')
    args = ap.parse_args()
    root = Path(args.dataset)
    splits = json.loads(Path(args.splits).read_text())
    rows = [r for r in csv.DictReader(open(root / 'index.csv')) if splits.get(r['recording']) == args.split]
    session = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    result = {'split': args.split, 'model': args.model, 'recordings': sorted({r['recording'] for r in rows}),
              'gt_crops': evaluate(session, rows, 'gt_crop', root),
              'depth_detector_crops': evaluate(session, rows, 'det_crop', root),
              'depth_detector_only': depth_report(rows)}
    text = json.dumps(result, indent=1)
    Path(args.out or Path(args.model).parent / f'eval_{args.split}.json').write_text(text)
    for key in ('gt_crops', 'depth_detector_crops'):
        e = result[key]
        if e is None:
            continue
        for level in ('per_crop', 'per_parcel'):
            m = e[level]
            print(f"{key:22s} {level:10s} n={m['n']:5d} acc={m['accuracy']:.3f} dmg_prec={m['damaged_precision']:.3f} "
                  f"dmg_recall={m['damaged_recall']:.3f} intact_recall={m['intact_recall']:.3f} "
                  f"confusion={m['confusion']}")
    print('depth detector only:', json.dumps(result['depth_detector_only'], indent=None))


if __name__ == '__main__':
    main()
