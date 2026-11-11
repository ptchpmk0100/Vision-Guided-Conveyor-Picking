#!/usr/bin/env python3
"""7/8/10-box profiles on the ORIGINAL belt: capacity, fixtures, perception and sensor identity."""
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import os
import numpy as np

from twin_moveit_scripts.parcel_layout import (batch_layout,PARCEL_SIZE,footprints_clear,fit_pitch,
    clearance_at,MIN_PITCH,GRIP_CLEARANCE,QUEUE_LAST_Y_LIMIT,BELT_DECK_Y)
from twin_moveit_scripts.parcel_batch_world import batch_world
from twin_moveit_scripts.parcel_grasp import parcel_from_collision,GraspEvaluator,GripSample
from twin_moveit_scripts.parcel_perception import detect_regions

ROOT=Path(__file__).resolve().parents[1]


class TenParcelTest(unittest.TestCase):
    def test_capacity_spacing_and_original_profile(self):
        b=batch_layout(10)
        self.assertEqual(len(set(b.slots)),10)
        self.assertEqual(b.names[-1],'parcel_10')
        self.assertTrue(all(b.contains(*xy) for xy in b.slots))
        self.assertEqual(batch_layout().slots,((.260,-.360),(.380,-.360),(.320,-.270)))
        self.assertEqual(batch_layout().pitch,1.2)
        for count in (7,8,10):
            b=batch_layout(count)
            self.assertEqual(len(b.slots),count)
            self.assertEqual(b.slots,batch_layout(10).slots[:count])
            self.assertEqual(b.pitch,fit_pitch(count))
            self.assertGreaterEqual(b.queue_y[-1],QUEUE_LAST_Y_LIMIT)
            self.assertGreaterEqual(b.queue_y[0]-b.queue_y[-1],0)
            self.assertAlmostEqual(b.queue_y[0]-b.queue_y[1],b.pitch)
            self.assertGreaterEqual(clearance_at(b.pitch),GRIP_CLEARANCE)
        self.assertEqual([batch_layout(n).pitch for n in (7,8,10)],[.516,.442,.344])
        for value in (0,.05,float('nan'),2,.9):
            with self.assertRaises(ValueError):batch_layout(10,value)
        with self.assertRaises(ValueError):batch_layout(10,MIN_PITCH-.001)
        with self.assertRaises(ValueError):batch_layout(12)
        for i,xy in enumerate(batch_layout(10).slots):
            for other in batch_layout(10).slots[i+1:]:self.assertTrue(footprints_clear((*xy,0),(*other,0)))
        self.assertFalse(footprints_clear((0,0,0),(.075,0,0)))
        self.assertTrue(footprints_clear((0,0,0),(.075,0,math.pi/2)))

    def test_minimum_pitch_leaves_gripper_clearance_beside_open_fingers(self):
        # Open finger outer edge is 66.4 mm from the box centre; the neighbour's
        # near face (yawed up to 15 degrees) must stay at least 35 mm further out.
        self.assertGreaterEqual(clearance_at(MIN_PITCH),GRIP_CLEARANCE)
        self.assertLess(clearance_at(MIN_PITCH-.005),GRIP_CLEARANCE)

    def test_ten_components_with_wide_intrinsics(self):
        b=batch_layout(10);f=640/(2*math.tan(b.camera_hfov/2))
        k=np.array([[f,0,320],[0,f,240],[0,0,1.]])
        r=np.array([[0.,-1,0],[-1.,0,0],[0,0,-1.]])
        depth=np.full((480,640),.905,np.float32)
        for x,y in b.slots:
            u0=math.ceil(320-(y+.025)*f/.855);u1=math.floor(320-(y-.025)*f/.855)
            v0=math.ceil(240-(x+.04-.4)*f/.855);v1=math.floor(240-(x-.04-.4)*f/.855)
            self.assertTrue(0<=u0<u1<640 and 0<=v0<v1<480)
            depth[v0:v1+1,u0:u1+1]=.855
        boxes=detect_regions(depth,k,r,np.array([.4,0,1.2]),[b.tote_region])[0]
        self.assertEqual(len(boxes),10)
        self.assertTrue(all(min(math.dist(p[:2],xy) for p in boxes)<.003 for xy in b.slots))

    def test_active_identity_ten_not_foreign_or_alias(self):
        with patch.dict(os.environ,{'TWIN_PARCEL_COUNT':'10','TWIN_PARCEL_PITCH':'.344'}):
            self.assertEqual(parcel_from_collision('parcel_10::body::carton'),'parcel_10')
            for name in ('parcel_11','parcel_0','parcel_01','parcel_10wrong'):
                self.assertIsNone(parcel_from_collision(name+'::body::carton'))
            evaluator=GraspEvaluator()
            for i in range(12):
                ready,_=evaluator.update(GripSample(round((1+i*.01)*1e9),(.025,.025),.020,
                    frozenset(['parcel_10']),frozenset(['parcel_10'])))
            self.assertTrue(ready)
        self.assertIsNone(parcel_from_collision('parcel_10::body::carton',batch_layout().names))

    def test_belt_is_never_lengthened_and_fixtures_clear(self):
        source=ROOT/'src/twin_action_demo/worlds/industrial_parcel_cell_continuous.sdf'
        original=batch_world(source,batch_layout(3))
        def belt_rows(root):
            rows=[]
            for name in ('pick_belt','conveyor_frame'):
                model=root.find(f"world/model[@name='{name}']")
                rows.append((name,model.findtext('pose')))
                for c in model.findall('link/collision'):
                    size=c.find('geometry/box/size')
                    cyl=c.find('geometry/cylinder')
                    rows.append((c.get('name'),c.findtext('pose'),None if size is None else size.text,
                                 None if cyl is None else (cyl.findtext('radius'),cyl.findtext('length'))))
            return rows
        for count in (7,8,10):
            b=batch_layout(count)
            root=batch_world(source,b)
            self.assertEqual(belt_rows(root),belt_rows(original),'belt/frame changed for %d boxes'%count)
            belt=root.find("world/model[@name='pick_belt']/link/collision[@name='deck']")
            center=list(map(float,belt.findtext('pose').split()))[0]
            length=float(belt.findtext('geometry/box/size').split()[0])
            self.assertAlmostEqual(length,BELT_DECK_Y[1]-BELT_DECK_Y[0])   # 5.0 m, as originally
            self.assertAlmostEqual(center-length/2,BELT_DECK_Y[0])
            self.assertLess(center-length/2,b.queue_y[-1]-.025)
            self.assertFalse([e for e in root.iter('collision') if (e.get('name') or '').startswith('batch_foot_')])
        b=batch_layout(10)
        root=batch_world(source,b)
        def boxes(name):
            model=root.find(f"world/model[@name='{name}']")
            pose=list(map(float,model.findtext('pose').split()));yaw=pose[5]
            r=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
            found=[]
            for c in model.findall('link/collision'):
                if c.find('geometry/box/size') is None:continue
                p=np.array(list(map(float,c.findtext('pose').split()))[:3])
                half=np.abs(r)@np.array(list(map(float,c.findtext('geometry/box/size').split())))/2
                p=np.array(pose[:3])+r@p;found.append((p-half,p+half))
            return found
        for name in ('tote_station','collection_tote_blue'):
            for low,high in boxes(name):
                for other in ('robot_pedestal','conveyor_frame'):
                    for a,z in boxes(other):
                        self.assertFalse(np.all(np.minimum(high,z)-np.maximum(low,a)>.0001),
                                         name+' intersects '+other)


if __name__=='__main__':unittest.main()
