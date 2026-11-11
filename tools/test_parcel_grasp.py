#!/usr/bin/env python3
"""Grasp decisions tested without a running ROS or Gazebo system."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/twin_moveit_scripts'))
from twin_moveit_scripts.parcel_grasp import GripSample, GraspEvaluator, close_command, parcel_from_collision


class GraspTests(unittest.TestCase):
    def sample(self, t, q=(.025, .025), desired=.020, left=('parcel_1',), right=('parcel_1',), **kw):
        return GripSample(round(t*1e9), q, desired, frozenset(left), frozenset(right), **kw)

    def confirm(self, evaluator):
        for i in range(12):
            state, _ = evaluator.update(self.sample(1+i*.01))
        self.assertTrue(state)

    def test_valid_grasp_and_seesaw(self):
        e = GraspEvaluator()
        for i in range(12):
            delta=.003 if i%2 else -.003
            state,_=e.update(self.sample(1+i*.01,(.025+delta,.025-delta)))
        self.assertTrue(state)

    def test_empty_hand_in_old_plausible_range_is_rejected(self):
        e=GraspEvaluator()
        for i in range(30):
            state,_=e.update(self.sample(1+i*.01,(.020,.020)))
        self.assertFalse(state)

    def test_loss_collapsing_to_command_is_detected(self):
        e=GraspEvaluator();self.confirm(e)
        for i in range(8):
            state,_=e.update(self.sample(1.2+i*.01,(.020,.020)))
        self.assertFalse(state)

    def test_single_pad_fixture_and_different_objects_never_confirm(self):
        for left,right in ((('parcel_1',),()),(('belt',),('belt',)),
                           (('parcel_1',),('parcel_2',)),(('parcel_1','foreign_contact'),('parcel_1',))):
            e=GraspEvaluator()
            for i in range(30):
                state,_=e.update(self.sample(1+i*.01,left=left,right=right))
            self.assertFalse(state)

    def test_transient_contact_flicker_debounces_but_loss_does_not(self):
        e=GraspEvaluator();self.confirm(e)
        self.assertTrue(e.update(self.sample(1.2,left=()))[0])
        self.assertTrue(e.update(self.sample(1.22))[0])
        for i in range(7): state,_=e.update(self.sample(1.3+i*.01,left=()))
        self.assertFalse(state)

    def test_stale_nan_and_duplicate_samples(self):
        for kw in ({'contact_age':.101},{'joint_age':.101},{'q':(float('nan'),.025)}):
            e=GraspEvaluator()
            for i in range(20):state,_=e.update(self.sample(1+i*.01,**kw))
            self.assertFalse(state)
        e=GraspEvaluator()
        for _ in range(50):state,_=e.update(self.sample(1.))
        self.assertFalse(state)
        e=GraspEvaluator();self.confirm(e)
        self.assertFalse(e.update(self.sample(1.11,joint_age=.101))[0])

    def test_current_desired_position_not_final_target(self):
        e=GraspEvaluator()
        for i in range(20):state,_=e.update(self.sample(1+i*.01,desired=.030))
        self.assertFalse(state)
        self.assertAlmostEqual(close_command(.5),.030)
        self.assertAlmostEqual(close_command(2),.020)

    def test_collision_names(self):
        self.assertEqual(parcel_from_collision('parcel_2::body::carton'),'parcel_2')
        self.assertIsNone(parcel_from_collision('display_carton_2::body::carton'))


if __name__=='__main__':unittest.main()
