#!/usr/bin/env python3
"""Make the golden test data for the C++ condition node from the PYTHON pipeline.

Reads a recorded bag (default: test recording r09), picks a few detection messages with their matching RGB frame,
and writes to --out (default src/twin_condition_cpp/test/fixtures):

  frame_N.png    lossless RGB frames (640x480)
  golden.json    camera intrinsics and extrinsic, crop-window cases (real + edge + rounding), crops, model input
                 values, logits and probabilities produced by parcel_condition.py and onnxruntime

Run with the ROS overlay and the training venv (needs rosbag2_py, onnxruntime, cv2):

    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=src/twin_moveit_scripts ~/venvs/parcel_cls/bin/python tools/make_condition_golden.py \
        --bag $BAG_ROOT/r09/bag --model logs/condition/runs/r1/model.onnx
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from twin_moveit_scripts.object_detector_sorting_line import _load_extrinsic
from twin_moveit_scripts.parcel_condition import (CLASSES, crop_window, cut_crop, preprocess, softmax,
                                                  world_to_pixel)

IMAGE, INFO, DETS = '/overhead_camera/image', '/overhead_camera/camera_info', '/parcel_cell/moving_detections'
MIN_CONF = 0.75


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    images, dets, info = [], [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (IMAGE, INFO, DETS):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == IMAGE and msg.encoding == 'rgb8':
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
            images.append((stamp(msg), rgb.reshape(msg.height, msg.width, 3).copy()))
        elif topic == DETS and msg.detections:
            dets.append(msg)
        elif topic == INFO and info is None:
            info = np.asarray(msg.k).reshape(3, 3)
    return images, dets, info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--extrinsic', default='src/twin_action_demo/config/overhead_camera_extrinsic.yaml')
    ap.add_argument('--out', default='src/twin_condition_cpp/test/fixtures')
    ap.add_argument('--frames', type=int, default=3)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rot, trans = _load_extrinsic(args.extrinsic)
    images, dets, k = read_bag(args.bag)
    session = ort.InferenceSession(args.model, providers=['CPUExecutionProvider'])
    model_sha = hashlib.sha256(Path(args.model).read_bytes()).hexdigest()

    # pick detection messages spread over the recording, preferring ones with several parcels
    idx = np.linspace(len(dets) * 0.2, len(dets) * 0.8, args.frames).astype(int)
    frames, window_cases = [], []
    for n, i in enumerate(idx):
        msg = dets[i]
        st = stamp(msg)
        ts, rgb = min(images, key=lambda f: abs(f[0] - st))
        assert abs(ts - st) <= 0.06, 'no matching frame'
        cv2.imwrite(str(out / f'frame_{n}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        crops, centres, kept = [], [], []
        for det in msg.detections:
            c = det.bbox.center.position
            centre = (c.x, c.y, c.z)
            win = crop_window(centre, k, rot, trans, rgb.shape)
            window_cases.append(dict(frame=n, centre=centre, window=None if win is None else list(win)))
            if win is None:
                continue
            crops.append(cut_crop(rgb, win))
            kept.append(centre)
        entry = dict(file=f'frame_{n}.png', detection_stamp=st, image_stamp=ts, detections=[])
        if crops:
            tensor = preprocess(crops)
            logits = session.run(None, {'input': tensor})[0]
            probs = softmax(logits)
            for centre, crop, t, lg, p in zip(kept, crops, tensor, logits, probs):
                best = int(p.argmax())
                entry['detections'].append(dict(
                    centre=centre, window=list(crop_window(centre, k, rot, trans, rgb.shape)),
                    crop_b64=base64.b64encode(crop.tobytes()).decode(),
                    tensor_head=[float(x) for x in t.reshape(-1)[:12]],
                    tensor_sum=float(t.sum()), logits=[float(x) for x in lg], probs=[float(x) for x in p],
                    label=CLASSES[best] if p[best] >= MIN_CONF else 'unknown', confidence=float(p[best])))
        frames.append(entry)

    # edge cases on the real camera: near each border, behind the camera
    centre_px = world_to_pixel((0.56, 0.0, 0.05), k, rot, trans)
    for name, centre in (('behind_camera', (0.56, 0.0, 1.5)), ('off_image_left', (0.56, -0.9, 0.05)),
                         ('off_image_right', (0.56, 0.9, 0.05)), ('near_centre', (0.40, 0.0, 0.05))):
        win = crop_window(centre, k, rot, trans, (480, 640))
        window_cases.append(dict(frame=None, name=name, centre=centre, window=None if win is None else list(win)))

    # rounding: identity camera, u = 100.5 -> corners 94.5 / 106.5; Python round() gives 94 / 106 (half to even)
    ident_k = np.array([[100.0, 0, 100.5], [0, 100.0, 100.5], [0, 0, 1.0]])
    eye, zero = np.eye(3), np.zeros(3)
    rounding = dict(intrinsics=ident_k.tolist(), rotation=eye.tolist(), translation=zero.tolist(), shape=[200, 200],
                    centre=[0.0, 0.0, 1.0], window=list(crop_window((0.0, 0.0, 1.0), ident_k, eye, zero, (200, 200))))

    golden = dict(
        note='Produced by tools/make_condition_golden.py from the Python pipeline. Do not edit by hand.',
        source_bag=str(Path(args.bag).parent.name), model_sha256=model_sha, min_confidence=MIN_CONF,
        cv2_version=cv2.__version__, onnxruntime_version=ort.__version__,
        classes=list(CLASSES), crop_size=64, input_size=128,
        intrinsics=k.tolist(), rotation=np.asarray(rot).tolist(), translation=np.asarray(trans).tolist(),
        image_shape=[480, 640], frames=frames, window_cases=window_cases, rounding_case=rounding,
        softmax_case=dict(logits=[[2.0, -1.0], [0.0, 0.0], [-3.0, 4.0]],
                          probs=softmax(np.array([[2.0, -1.0], [0.0, 0.0], [-3.0, 4.0]])).tolist()))
    (out / 'golden.json').write_text(json.dumps(golden, indent=1))
    n_det = sum(len(f['detections']) for f in frames)
    print(f'wrote {len(frames)} frames, {n_det} detections, {len(window_cases)} window cases to {out}')
    print('rounding case window:', rounding['window'], '(Python half-to-even; lround would give [95, 95, 107, 107])')
    print('sizes (KB):', {p.name: p.stat().st_size // 1024 for p in out.iterdir()})


if __name__ == '__main__':
    main()
