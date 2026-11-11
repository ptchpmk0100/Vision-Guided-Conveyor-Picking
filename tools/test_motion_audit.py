#!/usr/bin/env python3
"""Boundary tests for experimental preflight and planner selection."""
import copy
import math
import os
import unittest
from unittest.mock import patch
from twin_moveit_scripts.motion_audit import (validate, metrics, request_options,
    interpolated_positions, limit_profile, ARM_JOINTS)


class AuditTest(unittest.TestCase):
    def setUp(self):
        self.names = ['a', 'b']
        self.limits = {n: dict(position_min=-2, position_max=2, velocity_max=1,
                               acceleration_max=2) for n in self.names}
        self.data = dict(joint_names=self.names, points=[
            dict(time=0, positions=[0, 0], velocities=[0, 0], accelerations=[0, 0]),
            dict(time=1, positions=[1, 0], velocities=[0, 0], accelerations=[0, 0])])

    def check(self, data=None, limits=None):
        return validate(data or self.data, limits or self.limits, self.names)

    def test_good_is_never_execution_authority(self):
        verdict = self.check()
        self.assertTrue(verdict['sampled_checks_passed'])
        self.assertFalse(verdict['execution_authorized'])
        self.assertIn('continuous_collision', verdict['not_checked'])
        self.assertEqual(metrics(self.data)['joint_path_length_rad'], 1)

    def test_invalid_structure_and_samples(self):
        for field, value in [('time', 0), ('time', -1), ('time', math.nan),
                             ('positions', [0]), ('positions', [math.inf, 0]),
                             ('velocities', []), ('velocities', [1.1, 0]),
                             ('accelerations', [0, 2.1]), ('positions', [3, 0])]:
            with self.subTest(field=field, value=value):
                data = copy.deepcopy(self.data)
                data['points'][1][field] = value
                self.assertFalse(self.check(data)['sampled_checks_passed'])

    def test_order_and_limits(self):
        data = copy.deepcopy(self.data)
        data['joint_names'] = list(reversed(self.names))
        self.assertFalse(self.check(data)['sampled_checks_passed'])
        for value in [None, math.inf, -1, 0]:
            limits = copy.deepcopy(self.limits)
            limits['a']['acceleration_max'] = value
            self.assertFalse(self.check(limits=limits)['sampled_checks_passed'])

    def test_planner_cannot_silently_fallback(self):
        with patch.dict(os.environ, {'TWIN_PLANNER_ID': 'missing'}):
            with self.assertRaises(ValueError):
                request_options({'RRTConnectkConfigDefault': {}})
        with patch.dict(os.environ, {'TWIN_PLANNER_ID': 'known', 'TWIN_PLANNING_TIME': 'nan'}):
            with self.assertRaises(ValueError):
                request_options({'known': {}})

    def test_stationary_hold_requires_measured_match(self):
        data = copy.deepcopy(self.data)
        data['points'] = data['points'][:1]
        self.assertFalse(self.check(data)['sampled_checks_passed'])
        self.assertTrue(validate(data, self.limits, self.names,
                                stationary_start=[0, 0])['sampled_checks_passed'])
        self.assertFalse(validate(data, self.limits, self.names,
                                 stationary_start=[.001, 0])['sampled_checks_passed'])
        data['points'][0]['velocities'] = [.01, 0]
        self.assertFalse(validate(data, self.limits, self.names,
                                 stationary_start=[0, 0])['sampled_checks_passed'])

    def test_spline_endpoints_and_budget(self):
        positions = list(interpolated_positions(self.data))
        self.assertEqual(positions[0], [0, 0])
        self.assertEqual(positions[-1], [1, 0])
        self.assertAlmostEqual(positions[len(positions)//2][0], .5)
        with self.assertRaises(ValueError):
            list(interpolated_positions(self.data, max_samples=2))

    def test_limit_profile_intersects_configured_envelopes(self):
        urdf = '<robot>'+''.join(f'<joint name="{name}"><limit lower="-2" upper="2" velocity="3"/>'
            '<safety_controller soft_lower_limit="-1.5" soft_upper_limit="1.5"/></joint>'
            for name in ARM_JOINTS)+'</robot>'
        cfg = {n:dict(min_position=-1, max_position=3, max_velocity=4, max_acceleration=8)
               for n in ARM_JOINTS}
        limits = limit_profile(urdf,cfg,.5,.5)
        self.assertEqual(limits[ARM_JOINTS[0]], dict(position_min=-1, position_max=1.5,
                                                   velocity_max=1.5, acceleration_max=4))
        with self.assertRaises(ValueError):
            limit_profile(urdf,cfg,2,.5)

    def test_malformed_input_is_a_rejection(self):
        for data in (None, [], dict(points=3), dict(points=[None]),
                     dict(points=[dict(time=0,positions=None)])):
            with self.subTest(data=data):
                self.assertFalse(validate(data,self.limits,self.names)['sampled_checks_passed'])
        self.assertFalse(validate(self.data,{'a':None},self.names)['sampled_checks_passed'])


if __name__ == '__main__':
    unittest.main()
