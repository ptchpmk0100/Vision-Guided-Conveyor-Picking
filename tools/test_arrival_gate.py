#!/usr/bin/env python3
"""Shadow arrival replay: stale feedback cannot manufacture completion."""
import math
import unittest
from twin_moveit_scripts.arrival_gate import ArrivalGate, ArrivalProfile


class ArrivalTest(unittest.TestCase):
    def setUp(self):
        self.gate = ArrivalGate(ArrivalProfile((.01, .01), (.02, .02),
                                               (False, True), .2, .1, 2))
        self.gate.start('command1', (0, -math.pi+.001), 0)

    def update(self, now=.1, **fields):
        data = dict(now=now, stamp=now, positions=[0, math.pi-.001],
                    velocities=[0, 0], result_command_id='command1', controller_succeeded=True)
        data.update(fields)
        return self.gate.update(**data)

    def test_arrival_requires_distinct_fresh_samples(self):
        self.assertEqual(self.update(), 'DWELLING')
        self.assertEqual(self.update(.15, stamp=.1), 'SAMPLE_NOT_INCREASING')
        self.assertEqual(self.update(.21), 'DWELLING')
        self.assertEqual(self.update(.32), 'ARRIVED')

    def test_controller_id_and_velocity(self):
        self.assertEqual(self.update(result_command_id='old'), 'CONTROLLER_UNCONFIRMED')
        self.assertEqual(self.update(.2, velocities=[.1, 0]), 'NOT_SETTLED')
        self.assertEqual(self.update(.3), 'DWELLING')

    def test_fault_stale_nonfinite_timeout_and_reset(self):
        self.assertEqual(self.update(stamp=-.2), 'STATE_STALE')
        self.assertEqual(self.update(.2, positions=[math.nan, 0]), 'STATE_NONFINITE')
        self.assertEqual(self.update(.3, velocities=None), 'STATE_MISSING')
        self.assertEqual(self.update(.4, stamp=.5), 'STATE_STALE')
        self.assertEqual(self.update(.1), 'CLOCK_RESET')
        self.setUp()
        self.assertEqual(self.update(fault=True), 'FAULT')
        self.setUp()
        self.assertEqual(self.update(2), 'TIMEOUT')

    def test_one_joint_outlier_and_gap_reset_dwell(self):
        self.assertEqual(self.update(positions=[.1, math.pi]), 'NOT_SETTLED')
        self.assertEqual(self.update(.2), 'DWELLING')
        self.assertEqual(self.update(.5), 'DWELLING')
        self.assertEqual(self.update(.61), 'ARRIVED')

    def test_invalid_profiles_cannot_weaken_predicate(self):
        with self.assertRaises(ValueError):
            ArrivalProfile((.01,), (.02,), ('false',), .2, .1, 2)
        with self.assertRaises(ValueError):
            ArrivalProfile((True,), (.02,), (False,), .2, .1, 2)


if __name__ == '__main__':
    unittest.main()
