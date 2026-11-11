"""Challenger: classify each depth-detected parcel as intact / damaged from its RGB crop (log only).

Subscribes /overhead_camera/image + camera_info + the depth detector's detections, cuts the same world-window
crop used in training, runs an ONNX model (CPU) and publishes /parcel_cell/condition (Detection3DArray, one
detection per parcel, class_id 'intact' | 'damaged' | 'unknown', score = confidence). The arm pipeline does
NOT read this topic. Needs onnxruntime: run with tools/run_condition_node.sh (training venv + ROS overlay).
"""
import collections
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from twin_moveit_scripts.object_detector_sorting_line import _load_extrinsic
from twin_moveit_scripts.parcel_condition import CLASSES, crop_window, cut_crop, preprocess, softmax

CONDITION_TOPIC = '/parcel_cell/condition'


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class ConditionNode(Node):
    def __init__(self):
        super().__init__('parcel_condition')
        import onnxruntime as ort
        model = self.declare_parameter('model', '').value
        extrinsic = self.declare_parameter('extrinsic_file', '').value
        det_topic = self.declare_parameter('detection_topic', '/parcel_cell/moving_detections').value
        self.min_conf = float(self.declare_parameter('min_confidence', 0.75).value)
        if not model or not extrinsic:
            raise RuntimeError('parameters model and extrinsic_file are required')
        self.session = ort.InferenceSession(model, providers=['CPUExecutionProvider'])
        self.rotation, self.translation = _load_extrinsic(extrinsic)
        self.k = None
        self.frames = collections.deque(maxlen=6)
        self.stats = dict(msgs=0, parcels=0, unmatched=0, skipped_edge=0, lat_ms=[])
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/overhead_camera/image', self.on_image,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Detection3DArray, det_topic, self.on_detections, 10)
        self.pub = self.create_publisher(Detection3DArray, CONDITION_TOPIC, 10)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'condition classifier ready: model={model} detections={det_topic} '
                               f'min_confidence={self.min_conf}')

    def on_info(self, msg):
        self.k = np.asarray(msg.k).reshape(3, 3)

    def on_image(self, msg):
        if msg.encoding != 'rgb8':
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
            msg.height, msg.width, 3)
        self.frames.append((stamp_of(msg), rgb))

    def on_detections(self, msg):
        if self.k is None or not msg.detections:
            return
        t0 = time.perf_counter()
        stamp = stamp_of(msg)
        frame = min(self.frames, key=lambda f: abs(f[0] - stamp), default=None)
        if frame is None or abs(frame[0] - stamp) > 0.06:
            self.stats['unmatched'] += 1
            return
        rgb = frame[1]
        crops, keep = [], []
        for i, det in enumerate(msg.detections):
            c = det.bbox.center.position
            window = crop_window((c.x, c.y, c.z), self.k, self.rotation, self.translation, rgb.shape)
            if window is None:
                self.stats['skipped_edge'] += 1
                continue
            crops.append(cut_crop(rgb, window))
            keep.append(i)
        out = Detection3DArray()
        out.header = msg.header
        if crops:
            probs = softmax(self.session.run(None, {'input': preprocess(crops)})[0])
            for i, p in zip(keep, probs):
                det = Detection3D()
                det.header = msg.detections[i].header
                det.id = msg.detections[i].id
                det.bbox = msg.detections[i].bbox
                best = int(p.argmax())
                label = CLASSES[best] if p[best] >= self.min_conf else 'unknown'
                # results[0] = decision (label or 'unknown', confidence); results[1:] = both class probabilities
                # in CLASSES order, so a consumer can apply its own threshold / vote rule.
                for name, prob in (('', None), *zip(CLASSES, p)):
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label if prob is None else name
                    hyp.hypothesis.score = float(p[best]) if prob is None else float(prob)
                    hyp.pose.pose = det.bbox.center
                    det.results.append(hyp)
                out.detections.append(det)
        self.pub.publish(out)
        self.stats['msgs'] += 1
        self.stats['parcels'] += len(out.detections)
        self.stats['lat_ms'].append((time.perf_counter() - t0) * 1000)

    def report(self):
        s = self.stats
        lat = s['lat_ms']
        if lat:
            self.get_logger().info(
                f"msgs={s['msgs']} parcels={s['parcels']} unmatched={s['unmatched']} edge={s['skipped_edge']} "
                f"latency_ms mean={np.mean(lat):.1f} p95={np.percentile(lat, 95):.1f} max={np.max(lat):.1f}")
            s['lat_ms'] = lat[-200:]


def main():
    rclpy.init()
    node = ConditionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
