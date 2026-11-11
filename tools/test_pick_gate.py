#!/usr/bin/env python3
"""Early detection-gated launch, overlapped slot confirmation, short release stroke."""
import os
import unittest
from unittest.mock import MagicMock, Mock, patch

from twin_moveit_scripts import move_to
from twin_moveit_scripts import parcel_confirmed
from twin_moveit_scripts.parcel_continuous import (ALIGN_TIME, EARLY_LAUNCH_Y, FAST_TIMING,
                                                   InterceptNotFeasible, ORIGINAL_TIMING,
                                                   next_target)
from twin_moveit_scripts.parcel_tracking import BeltTracker
from twin_moveit_scripts.tote_slot import SlotConfirmation


def feed(tracker, speed, y0, seconds, step=.1, t0=0.):
    """Camera samples of one box moving at `speed`, starting at y0 (m)."""
    t = t0
    while t <= t0+seconds+1e-9:
        tracker.update(t, [(.56, y0+speed*(t-t0), .32)])
        t += step
    return t-step


class WindowTest(unittest.TestCase):
    def test_default_window_is_unchanged_and_early_window_launches_sooner(self):
        speed = .04
        tracker = BeltTracker(speed)
        now = feed(tracker, speed, -.34, .8)          # box ~ -0.31 m, track mature
        self.assertIsNone(tracker.nearest(now, (.56, .06, .52)))        # original window
        found = tracker.nearest(now, (.56, .06, .52), (EARLY_LAUNCH_Y-.1, -.075))
        self.assertIsNotNone(found)
        self.assertEqual(found[1], speed)

    def test_early_window_still_rejects_box_too_far_upstream(self):
        tracker = BeltTracker(.04)
        now = feed(tracker, .04, -.364, .6)           # y ~ -0.34, upstream of -0.25
        self.assertIsNone(tracker.nearest(now, (.56, .06, .52), (EARLY_LAUNCH_Y, -.075)))

    def test_late_box_still_fail_stops(self):
        tracker = BeltTracker(.04)
        now = feed(tracker, .04, -.2, 3.5)            # passes -0.075
        with self.assertRaisesRegex(RuntimeError, 'passed'):
            tracker.nearest(now, (.56, .06, .52), (EARLY_LAUNCH_Y, -.075))

    def test_persistent_tracker_selects_without_rebuilding(self):
        observed = MagicMock()
        observed.tracker = object()
        observed.now.return_value = 5.
        observed.tracked_target.return_value = ('track', .04)
        self.assertEqual(next_target(observed, .04, window=(EARLY_LAUNCH_Y, -.075)), ('track', .04))
        observed.tracked_target.assert_called_once()
        observed.current.assert_not_called()          # no per-cycle camera rebuild


class TimingProfileTest(unittest.TestCase):
    def test_original_three_box_timeline_is_untouched(self):
        self.assertEqual((ORIGINAL_TIMING.lead, ORIGINAL_TIMING.grasp, ORIGINAL_TIMING.lift), (.5, 1.6, 1.3))
        import inspect
        self.assertIs(inspect.signature(parcel_confirmed.intercept).parameters['timing'].default,
                      ORIGINAL_TIMING)

    def test_fast_timeline_keeps_planning_lead_and_a_real_descent(self):
        self.assertGreaterEqual(FAST_TIMING.lead, .25)                # check needs >= .15 s after planning
        self.assertGreaterEqual(FAST_TIMING.grasp-ALIGN_TIME, .5)     # 0.1 m descent stays gentle
        self.assertLess(FAST_TIMING.grasp, ORIGINAL_TIMING.grasp)
        self.assertGreaterEqual(FAST_TIMING.lift, .8)


class SlotConfirmationTest(unittest.TestCase):
    def test_confirms_after_continuous_window(self):
        rule = SlotConfirmation(.37, -.37, started=0.)
        results = [rule.update(True, .1*i, .1*i) for i in range(1, 9)]
        self.assertEqual(results.index(True), 6)      # first valid 0.1, confirmed at 0.7 (0.6 s later)
        self.assertFalse(any(results[:6]))

    def test_gap_or_invalid_sample_restarts_window(self):
        rule = SlotConfirmation(.37, -.37, started=0.)
        self.assertFalse(rule.update(True, 1., 1.))
        self.assertFalse(rule.update(True, 2., 2.))   # 1 s gap: must NOT confirm
        self.assertFalse(rule.update(True, 2.2, 2.2))
        self.assertFalse(rule.update(False, 2.3, 2.3))
        self.assertFalse(rule.update(True, 2.4, 2.4))
        self.assertFalse(rule.update(True, 2.65, 2.65))
        self.assertFalse(rule.update(True, 2.9, 2.9))     # only 0.5 s since the reset
        self.assertTrue(rule.update(True, 3.05, 3.05))

    def test_timeout_is_a_fault(self):
        rule = SlotConfirmation(.37, -.37, started=0.)
        with self.assertRaisesRegex(RuntimeError, 'did not settle'):
            rule.update(False, None, 13.)


class ReleaseStrokeTest(unittest.TestCase):
    def test_finger_goal_uses_requested_stroke_time(self):
        client, executor, twin, logger = (Mock() for _ in range(4))
        goals = []
        def capture(goal):
            goals.append(goal)
            raise RuntimeError('stop after capture')
        client.send_goal_async.side_effect = capture
        with patch.dict(os.environ, dict(TWIN_PARCEL_COUNT='3')):
            for seconds, expected in ((None, 2.5), (.8, .8)):
                with self.assertRaises(RuntimeError):
                    move_to._send_finger_goal(client, executor, twin, .04, logger, seconds=seconds)
                point = goals[-1].trajectory.points[0].time_from_start
                self.assertAlmostEqual(point.sec+point.nanosec/1e9, expected, places=6)

    def test_ten_deadline_budget_follows_the_stroke(self):
        class Future:
            def __init__(self):
                self.n = 0
            def done(self):
                return False
        executor, clock, future = Mock(), Mock(), Future()
        clock.sample = None
        samples = iter([100., 100.5, 101., 102., 103., 105.])
        def spin(timeout_sec):
            clock.sample = next(samples)
        executor.spin_once.side_effect = spin
        ticks = iter(range(100))
        with patch.object(move_to.time, 'monotonic', side_effect=lambda: next(ticks)*.2):
            error = move_to._wait_ten_finger_result(executor, Mock(), future, clock, .8)
        self.assertIn('simulation deadline', error)   # 0.8 s stroke + 2 s margin = 2.8 sim-s
        self.assertGreaterEqual(clock.sample-100., 2.8)
        self.assertLess(clock.sample-100., 4.5)


class PlaceStageTest(unittest.TestCase):
    def run_place(self, **kwargs):
        grip = move_to.GripResult(ok=True, stage='reached', stalled=False, final_width=.04)
        reached = Mock(ok=True)
        with patch.object(move_to, '_GRASP_STATE', dict(known=True, width=.025)), \
             patch.object(move_to, '_read_finger_width', return_value=.025), \
             patch.object(move_to, '_grasp_goal_state', return_value=Mock()), \
             patch.object(move_to, '_plan_chain_execute', return_value=reached), \
             patch.object(move_to, 'move_gripper', return_value=grip) as gripper, \
             patch.object(move_to, 'move_to_grasp_pose', return_value=reached) as pose:
            result = move_to.place(Mock(), Mock(), Mock(), Mock(), Mock(), (.37, -.37, .32),
                                   clearance=.09, home_first=False, blend=True, **kwargs)
        return result, gripper, pose

    def test_default_place_keeps_retract_and_default_stroke(self):
        result, gripper, pose = self.run_place()
        self.assertTrue(result.ok)
        pose.assert_called_once()                       # the retract
        self.assertIsNone(gripper.call_args.kwargs['seconds'])

    def test_extended_place_skips_retract_and_uses_short_stroke(self):
        result, gripper, pose = self.run_place(release_seconds=.8, retract=False)
        self.assertTrue(result.ok)
        self.assertTrue(result.released)
        pose.assert_not_called()
        self.assertEqual(gripper.call_args.kwargs['seconds'], .8)


if __name__ == '__main__':
    unittest.main()
