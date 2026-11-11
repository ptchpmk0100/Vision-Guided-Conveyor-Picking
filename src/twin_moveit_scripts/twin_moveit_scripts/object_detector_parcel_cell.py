"""Publish fresh parcel detections from calibrated RGBD depth; no simulator poses."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from twin_moveit_scripts.extrinsic import load_extrinsic as _load_extrinsic
from twin_moveit_scripts.parcel_layout import DETECTION_TOPIC, MOVING_DETECTION_TOPIC, PARCEL_SIZE,batch_layout
from twin_moveit_scripts.parcel_perception import detect_regions


class ParcelDetector(Node):
    def __init__(self):
        super().__init__('object_detector_parcel_cell')
        count=self.declare_parameter('parcel_count',3).value
        pitch=self.declare_parameter('parcel_pitch',batch_layout(count).pitch).value
        self.batch=batch_layout(count,pitch)
        self.declare_parameter('batch_scene_hash','')
        path = self.declare_parameter('extrinsic_file', '').value
        if not path:
            raise RuntimeError('parcel detector requires a calibrated extrinsic_file')
        self.rotation, self.translation = _load_extrinsic(path)
        self.k = None
        self.depth = None
        self.processed_stamp = None
        self.region_detections = None
        self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/overhead_camera/depth_image', self.image,
                                 QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(Detection3DArray, DETECTION_TOPIC, 10)
        self.tote_pub = self.create_publisher(Detection3DArray, '/parcel_cell/tote_detections', 10)
        self.moving_pub = (self.create_publisher(Detection3DArray, MOVING_DETECTION_TOPIC, 10)
                           if self.declare_parameter('continuous', False).value else None)
        self.get_logger().info(f'Calibrated depth parcel detector: {path}')

    def info(self, msg):
        self.k = np.asarray(msg.k).reshape(3, 3)

    def image(self, msg):
        if msg.encoding != '32FC1' or msg.is_bigendian:
            self.get_logger().error('Unsupported depth encoding/endian')
            return
        self.depth = msg
        self.tick()

    def tick(self):
        if self.k is None or self.depth is None:
            return
        msg = self.depth
        stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        publishers = [self.pub, self.tote_pub]
        regions = [(.505, .615, -.180, .180), self.batch.tote_region]
        if self.moving_pub is not None:
            publishers.append(self.moving_pub)
            regions.append((.505, .615, -.400, .180))
        if 0 <= age <= 1.0:
            if stamp != self.processed_stamp:
                depth = np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                                   strides=(msg.step, 4))
                self.region_detections = detect_regions(depth, self.k, self.rotation,
                                                        self.translation, regions)
                self.processed_stamp = stamp
            detections = self.region_detections
        else:
            detections = [[] for _ in regions]
        for publisher, boxes in zip(publishers, detections):
            self.publish_region(msg, publisher, boxes)

    def publish_region(self, msg, publisher, boxes):
        array = Detection3DArray()
        array.header = msg.header
        array.header.frame_id = 'world'
        for i, (x, y, z) in enumerate(boxes):
            det = Detection3D()
            det.header = array.header
            det.id = f'observation_{i}'
            det.bbox.center.position.x = x
            det.bbox.center.position.y = y
            det.bbox.center.position.z = z
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = PARCEL_SIZE
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = 'parcel_box'
            result.hypothesis.score = 1.0
            result.pose.pose = det.bbox.center
            det.results.append(result)
            array.detections.append(det)
        publisher.publish(array)


def main():
    rclpy.init()
    node = ParcelDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
