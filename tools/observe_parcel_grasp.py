#!/usr/bin/env python3
"""Independent actual-joint, world-pose and pad-contact evidence; no command inputs."""
import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection3DArray
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from twin_moveit_scripts.parcel_contacts import CONTACT_TOPICS
from twin_moveit_scripts.parcel_layout import MOVING_DETECTION_TOPIC,active_batch


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    names=frozenset(active_batch().names)
    rclpy.init()
    ros=rclpy.create_node('independent_grasp_observer')
    transport=Node()
    pending=[]
    lock=threading.Lock()
    last_joint=-1
    latest_pose_ns=0

    def put(row):
        row['wall']=time.monotonic()
        with lock:pending.append(row)

    def joint(msg):
        nonlocal last_joint
        ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        if ns-last_joint < 10000000:return
        last_joint=ns
        put({'kind':'joints','ns':ns,'names':list(msg.name),'q':list(msg.position),'v':list(msg.velocity)})

    def poses(msg):
        nonlocal latest_pose_ns
        ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nsec
        latest_pose_ns=ns
        for p in msg.pose:
            if p.name in names:
                put({'kind':'pose','ns':ns,'name':p.name,'xyz':[p.position.x,p.position.y,p.position.z],
                     'quat':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]})

    def contacts(msg,side):
        put({'kind':'contact','side':side,'ns':msg.header.stamp.sec*1000000000+msg.header.stamp.nsec,
             'pairs':[[c.collision1.name,c.collision2.name] for c in msg.contact]})

    def detection(msg):
        # Independent record of the DETECTOR's publish cadence (sim stamp + wall).
        # Compared against the child's recv_sim_gaps it localises any receipt gap to
        # the detector (publish stall) vs the child subscription (delivery stall).
        put({'kind':'detection','ns':msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
             'n':len(msg.detections)})

    sub=ros.create_subscription(JointState,'/joint_states',joint,10)
    subd=ros.create_subscription(Detection3DArray,MOVING_DETECTION_TOPIC,detection,10)
    assert transport.subscribe(Pose_V,'/world/empty/dynamic_pose/info',poses)
    assert transport.subscribe(Image,'/overhead_camera/depth_image',lambda msg:put(
        {'kind':'camera_age','ns':latest_pose_ns,'source_ns':msg.header.stamp.sec*1000000000+msg.header.stamp.nsec}))
    for side in ('left','right'):
        assert transport.subscribe(Contacts,CONTACT_TOPICS[side],lambda msg,s=side:contacts(msg,s))
    try:
        with args.output.open('a') as out:
            while rclpy.ok():
                rclpy.spin_once(ros,timeout_sec=.01)
                with lock:
                    batch=pending[:];pending.clear()
                for row in batch:out.write(json.dumps(row,separators=(',',':'))+'\n')
                if batch:out.flush()
    except KeyboardInterrupt:pass
    finally:
        ros.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
