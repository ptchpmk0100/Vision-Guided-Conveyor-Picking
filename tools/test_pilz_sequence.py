#!/usr/bin/env python3
"""Sequence contracts prevent blended contact stops and overlapping spheres."""
import copy
import unittest
from moveit_msgs.msg import MotionSequenceItem
from twin_moveit_scripts.pilz_sequence import sequence_errors


class SequenceTest(unittest.TestCase):
    def setUp(self):
        self.items=[MotionSequenceItem(),MotionSequenceItem()]
        for item in self.items:item.req.group_name='panda_arm'
        self.items[0].req.start_state.joint_state.name=['panda_joint1']
        self.items[0].req.start_state.joint_state.position=[0.]
        self.items[0].blend_radius=.005
        self.xyz=[[0,0,0],[.05,0,0]]

    def test_valid_and_contact_stop(self):
        self.assertEqual(sequence_errors(self.items,self.xyz,[False,True]),[])
        self.assertIn('CONTACT_MUST_STOP:0',sequence_errors(self.items,self.xyz,[True,True]))

    def test_start_group_radius_overlap(self):
        items=copy.deepcopy(self.items)
        items[1].req.start_state.is_diff=True
        self.assertIn('START_STATE_OWNERSHIP:1',sequence_errors(items,self.xyz,[False,True]))
        items=copy.deepcopy(self.items);items[-1].blend_radius=.001
        self.assertIn('FINAL_RADIUS_NONZERO',sequence_errors(items,self.xyz,[False,False]))
        self.assertIn('BLEND_SPHERES_OVERLAP:0',
                      sequence_errors(self.items,[[0,0,0],[.001,0,0]],[False,True]))
        items=copy.deepcopy(self.items);items[1].req.group_name='other'
        self.assertIn('GROUP_MISMATCH',sequence_errors(items,self.xyz,[False,True]))


if __name__=='__main__':unittest.main()
