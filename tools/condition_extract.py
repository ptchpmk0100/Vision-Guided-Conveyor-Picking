#!/usr/bin/env python3
"""Bags -> labelled crops (ground-truth crops + depth-detector crops), index.csv, dataset manifest.

Run with the ROS python (rosbag2_py), NOT the training venv.
Labels come from the variant manifest written at spawn time (condition by construction); crop positions come
from Gazebo parcel poses projected through the calibrated camera. The depth detector's own detections are cut
the same way so the real pipeline (depth box -> RGB crop) can be evaluated separately.
"""
import argparse
import bisect
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3DArray

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.object_detector_sorting_line import _load_extrinsic  # noqa: E402
from twin_moveit_scripts.parcel_condition import (CROP_SIZE, CROP_WINDOW_M, crop_window, cut_crop,  # noqa: E402
                                                  parcel_corners, world_to_pixel)
from twin_moveit_scripts.parcel_layout import PARCEL_Z  # noqa: E402

EXTRINSIC = ROOT / 'src/twin_action_demo/config/overhead_camera_extrinsic.yaml'
DET_TOPIC = '/parcel_cell/moving_detections'


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def open_reader(bag_dir):
    """MCAP bag, compressed or not."""
    compressed = any(Path(bag_dir).glob('*.zstd'))
    reader = rosbag2_py.SequentialCompressionReader() if compressed else rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    return reader


def read_bag(bag_dir):
    """Pass 1: small topics only. Returns gt, dets, camera_info and the list of RGB (bag_index, stamp)."""
    reader = open_reader(bag_dir)
    gt, dets, info, frames = [], [], None, []
    n = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/gt/parcel_poses':
            msg = deserialize_message(data, TFMessage)
            t = msg.transforms[0].header.stamp
            gt.append((t.sec + t.nanosec * 1e-9, {
                x.child_frame_id: (x.transform.translation.x, x.transform.translation.y,
                                   x.transform.translation.z, yaw_of(x.transform.rotation))
                for x in msg.transforms}))
        elif topic == DET_TOPIC:
            msg = deserialize_message(data, Detection3DArray)
            dets.append((stamp_of(msg), [(d.bbox.center.position.x, d.bbox.center.position.y,
                                          d.bbox.center.position.z) for d in msg.detections]))
        elif topic == '/overhead_camera/camera_info' and info is None:
            info = deserialize_message(data, CameraInfo)
        elif topic == '/overhead_camera/image':
            frames.append((n, stamp_of(deserialize_message(data, Image))))
            n += 1
    return gt, dets, info, frames


def nearest(times, stamp, tolerance):
    i = bisect.bisect_left(times, stamp)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(times) and abs(times[j] - stamp) <= tolerance and (
                best is None or abs(times[j] - stamp) < abs(times[best] - stamp)):
            best = j
    return best


def extract(rec_dir, out_dir, gap, tol_gt=0.08, check_every=12):
    manifest = json.loads((rec_dir / 'manifest.json').read_text())
    cond = {p['name']: p['condition'] for p in manifest['parcels']}
    gt, dets, info, frames = read_bag(rec_dir / 'bag')
    if not gt or info is None or not frames:
        raise RuntimeError(f'{rec_dir.name}: incomplete bag (gt={len(gt)} info={info is not None} rgb={len(frames)})')
    gt_t = [g[0] for g in gt]
    det_t = [d[0] for d in dets]
    k = np.asarray(info.k).reshape(3, 3)
    rotation, translation = _load_extrinsic(str(EXTRINSIC))
    wanted, last, sampled = {}, -1e9, 0
    for index, stamp in frames:
        if stamp - last >= gap:
            wanted[index] = stamp
            last = stamp
    reader = open_reader(rec_dir / 'bag')
    rows, n = [], 0
    for cls in ('intact', 'damaged'):
        (out_dir / 'crops' / rec_dir.name / cls).mkdir(parents=True, exist_ok=True)
    (out_dir / 'check').mkdir(parents=True, exist_ok=True)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/overhead_camera/image':
            continue
        index, n = n, n + 1
        if index not in wanted:
            continue
        stamp = wanted[index]
        gi = nearest(gt_t, stamp, tol_gt)
        if gi is None:
            continue
        msg = deserialize_message(data, Image)
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3)
        di = nearest(det_t, stamp, 0.06)
        detections = dets[di][1] if di is not None else []
        sampled += 1
        draw = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy() if sampled % check_every == 1 else None
        for name, (x, y, z, yaw) in sorted(gt[gi][1].items()):
            if abs(z - PARCEL_Z) > 0.02:
                continue                                    # fell or tilted off the belt
            others = [(ox - x, oy - y) for o, (ox, oy, _, _) in gt[gi][1].items() if o != name]
            if min((math.hypot(a, b) for a, b in others), default=9) < 0.12:
                continue                                    # touching / queued at the end stop: not a belt view
            centre = (x, y, z)
            window = crop_window(centre, k, rotation, translation, rgb.shape)
            if window is None:
                continue
            label = cond[name]
            stem = f'{rec_dir.name}_f{sampled:03d}_{name}'
            gt_path = Path('crops') / rec_dir.name / label / f'{stem}_gt.png'
            cv2.imwrite(str(out_dir / gt_path), cv2.cvtColor(cut_crop(rgb, window), cv2.COLOR_RGB2BGR))
            row = dict(recording=rec_dir.name, frame=sampled, stamp=round(stamp, 3), parcel=name, label=label,
                       x=round(x, 4), y=round(y, 4), z=round(z, 4), yaw_deg=round(math.degrees(yaw), 2),
                       gt_crop=str(gt_path), det_found=0, det_crop='', det_err_mm='')
            match = [(math.hypot(d[0] - x, d[1] - y), d) for d in detections]
            match = min(match, default=None)
            if match is not None and match[0] < 0.03:
                dwin = crop_window(match[1], k, rotation, translation, rgb.shape)
                if dwin is not None:
                    det_path = Path('crops') / rec_dir.name / label / f'{stem}_det.png'
                    cv2.imwrite(str(out_dir / det_path),
                                cv2.cvtColor(cut_crop(rgb, dwin), cv2.COLOR_RGB2BGR))
                    row.update(det_found=1, det_crop=str(det_path), det_err_mm=round(match[0] * 1000, 1))
            rows.append(row)
            if draw is not None:
                colour = (0, 0, 255) if label == 'damaged' else (0, 200, 0)
                pts = [world_to_pixel(c, k, rotation, translation) for c in parcel_corners(centre, yaw)]
                hull = cv2.convexHull(np.array([[p[0], p[1]] for p in pts], np.float32)).astype(np.int32)
                cv2.polylines(draw, [hull], True, colour, 1)
                cv2.rectangle(draw, window[:2], window[2:], (255, 255, 0), 1)
                cv2.putText(draw, f'{name[7:]}{label[0]}', (window[0], window[1] - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)
        if draw is not None:
            for d in detections:
                p = world_to_pixel(d, k, rotation, translation)
                if p:
                    cv2.drawMarker(draw, (int(p[0]), int(p[1])), (0, 255, 255), cv2.MARKER_CROSS, 9, 1)
            cv2.imwrite(str(out_dir / 'check' / f'{rec_dir.name}_f{sampled:03d}.jpg'), draw)
    return rows, manifest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--recordings', required=True, help='dir containing one sub-dir per recording')
    ap.add_argument('--out', required=True)
    ap.add_argument('--gap', type=float, default=1.0, help='minimum sim seconds between sampled frames')
    ap.add_argument('--only', nargs='*', help='recording names (default: all with a manifest)')
    args = ap.parse_args()
    root, out = Path(args.recordings), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    names = args.only or sorted(p.name for p in root.iterdir() if (p / 'manifest.json').exists()
                                and (p / 'bag').exists())
    all_rows, recs = [], []
    for name in names:
        rows, manifest = extract(root / name, out, args.gap)
        summary = json.loads((root / name / 'run_summary.json').read_text()) \
            if (root / name / 'run_summary.json').exists() else {}
        all_rows += rows
        recs.append({'name': name, 'seed': manifest['seed'], 'belt_speed': summary.get('speed'),
                     'parcels_damaged': sum(p['condition'] == 'damaged' for p in manifest['parcels']),
                     'parcels': len(manifest['parcels']), 'crops': len(rows),
                     'crops_damaged': sum(r['label'] == 'damaged' for r in rows),
                     'det_crops': sum(r['det_found'] for r in rows)})
        print(f'{name}: {len(rows)} crops, {recs[-1]["det_crops"]} with a depth detection')
    with open(out / 'index.csv', 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    commit = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    (out / 'dataset_manifest.json').write_text(json.dumps({
        'git_commit': commit, 'crop_window_m': CROP_WINDOW_M, 'crop_size': CROP_SIZE, 'sample_gap_sim_s': args.gap,
        'classes': ['intact', 'damaged'], 'label_source': 'variant manifest (spawn-time condition) + Gazebo poses',
        'total_crops': len(all_rows), 'crops_per_class': {c: sum(r['label'] == c for r in all_rows)
                                                          for c in ('intact', 'damaged')},
        'recordings': recs}, indent=1))
    print(f'total {len(all_rows)} crops -> {out}')


if __name__ == '__main__':
    main()
