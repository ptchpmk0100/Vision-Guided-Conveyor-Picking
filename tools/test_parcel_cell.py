#!/usr/bin/env python3
"""Parcel geometry and depth segmentation checks without ROS/Gazebo processes."""
from pathlib import Path
import sys
import os
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.parcel_layout import SLOTS, inside_tote
from twin_moveit_scripts.parcel_perception import detect
from twin_moveit_scripts.parcel_task_session import task_session
from twin_moveit_scripts.parcel_tracking import BeltTracker, follow_distance
from check_continuous_transport import verdict as continuous_verdict


class ContinuousEvidenceTests(unittest.TestCase):
    def evidence(self):
        task = {'belt_speed': .06, 'events': [
            {'state': 'BELT_RUNNING', 'sim_time': 1.},
            *[{'state': 'INTERCEPT', 'start_sim': t, 'target_at_start': [.56, -.3+.06*(t-1), .32],
               'velocity': .06, 'grasp_sim': t+1.2, 'lift_sim': t+2.6} for t in (5., 7., 9.)],
            {'state': 'FULL', 'sim_time': 14.}]}
        evidence = {'belt': [[float(t), .06] for t in np.arange(1.2, 13.81, .1)],
                    'poses': [[float(t), f'parcel_{i}', .56, -.3+.06*(t-1), .32]
                              for i in (1, 2, 3) for t in np.arange(1.2, 13.81, .1)]}
        return task, evidence

    def test_continuous_command_and_actual_motion_required(self):
        task, evidence = self.evidence()
        self.assertTrue(continuous_verdict(task, evidence)['passed'])
        evidence['belt'][30][1] = 0.
        self.assertFalse(continuous_verdict(task, evidence)['passed'])
        task, evidence = self.evidence()
        for row in evidence['poses']:
            if row[1] == 'parcel_1' and 6.3 <= row[0] <= 7.5:
                row[3] = -.3+.06*(6.3-1)
        self.assertFalse(continuous_verdict(task, evidence)['passed'])
        task, evidence = self.evidence()
        for row in evidence['poses']:
            row[3] = -.3
        self.assertFalse(continuous_verdict(task, evidence)['passed'])


class TrackingTests(unittest.TestCase):
    def test_following_stops_smoothly_after_grasp_and_lift(self):
        for t in (0., 1., 2., 3., 4.1):
            self.assertAlmostEqual(follow_distance(t, .06), .06*t)
        self.assertAlmostEqual(follow_distance(4.5, .06), .258)
        near_end_speed = (follow_distance(4.5, .06)-follow_distance(4.499, .06))/.001
        self.assertLess(near_end_speed, .00001)

    def test_camera_velocity_prediction_and_duplicate_stamp(self):
        tracker = BeltTracker(.06)
        for stamp in (1., 1.1, 1.2, 1.3, 1.4):
            tracker.update(stamp, [(.56, -.14+.06*(stamp-1), .32)])
        tracker.update(1.4, [(.56, .1, .32)])
        track, velocity = tracker.nearest(1.45, (.56, .06, .52))
        self.assertAlmostEqual(velocity, .06)
        self.assertAlmostEqual(track.predict(1.6, velocity)[1], -.104)
        self.assertIsNone(tracker.nearest(1.7, (.56, .06, .52)))

    def test_stopped_wrong_speed_and_late_parcels_are_rejected(self):
        for speed, first_y in ((0., -.14), (.06, -.06)):
            tracker = BeltTracker(.06)
            for stamp in (1., 1.1, 1.2, 1.3, 1.4):
                tracker.update(stamp, [(.56, first_y+speed*(stamp-1), .32)])
            if speed == 0.:
                self.assertIsNone(tracker.nearest(1.4, (.56, .06, .52)))
            else:
                with self.assertRaises(RuntimeError):
                    tracker.nearest(1.4, (.56, .06, .52))

    def test_nearest_reachable_of_two_tracks(self):
        tracker = BeltTracker(.06)
        for stamp in (1., 1.1, 1.2, 1.3, 1.4):
            tracker.update(stamp, [(.56, y+.06*(stamp-1), .32) for y in (-.14, -.10)])
        track, _ = tracker.nearest(1.4, (.56, .06, .52))
        self.assertAlmostEqual(track.samples[-1][1][1], -.076)

    def test_roi_clipped_centroids_do_not_bias_velocity(self):
        tracker = BeltTracker(.06)
        for stamp, y in ((1., -.39), (1.1, -.385), (1.2, -.38), (1.3, -.37)):
            tracker.update(stamp, [(.56, y, .32)])
        self.assertEqual(tracker.tracks, [])
        for stamp in (1.4, 1.5, 1.6, 1.7, 1.8):
            tracker.update(stamp, [(.56, -.14+.06*(stamp-1.4), .32)])
        _, velocity = tracker.nearest(1.8, (.56, .06, .52))
        self.assertAlmostEqual(velocity, .06)

    def test_tracks_upstream_before_waiting_for_intercept_window(self):
        tracker = BeltTracker(.06)
        for stamp in np.arange(1., 4.01, .1):
            tracker.update(float(stamp), [(.56, -.3+.06*(stamp-1), .32)])
            found = tracker.nearest(float(stamp), (.56, .06, .52))
            if stamp < 3.3:
                self.assertIsNone(found)
        self.assertIsNotNone(found)

    def test_invalid_speed_and_continuous_belt_support(self):
        for speed in (float('nan'), -.1, .1):
            with self.assertRaises(ValueError):
                BeltTracker(speed)
        root = ET.parse(ROOT/'src/twin_action_demo/worlds/industrial_parcel_cell_continuous.sdf')
        deck = root.find("world/model[@name='pick_belt']/link/collision[@name='deck']")
        x = float(deck.findtext('pose').split()[0])
        length = float(deck.findtext('geometry/box/size').split()[0])
        self.assertLessEqual(x-length/2, -3.6)


class SessionTests(unittest.TestCase):
    def test_same_domain_is_exclusive_and_release_allows_retry(self):
        with tempfile.TemporaryDirectory() as folder:
            with task_session(93, folder):
                with self.assertRaisesRegex(RuntimeError, 'already running'):
                    with task_session(93, folder):
                        self.fail('Overlapping task acquired the same controller domain')
                with task_session(94, folder):
                    pass
            with task_session(93, folder):
                pass

    def test_process_exit_releases_the_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            env = os.environ.copy()
            env['PYTHONPATH'] = str(ROOT / 'src/twin_moveit_scripts')
            child = subprocess.run([sys.executable, '-c',
                'import os,sys; from twin_moveit_scripts.parcel_task_session import task_session; '
                'guard=task_session(93,sys.argv[1]); guard.__enter__(); os._exit(0)', folder],
                env=env, timeout=5)
            self.assertEqual(child.returncode, 0)
            with task_session(93, folder):
                pass


class ParcelTests(unittest.TestCase):
    def setUp(self):
        self.k = np.array([[550., 0, 320], [0, 550., 240], [0, 0, 1]])
        self.rotation = np.array([[0., -1, 0], [-1., 0, 0], [0, 0, -1.]])
        self.translation = np.array([.4, 0, 1.2])
        self.depth = np.full((480, 640), .905, dtype=np.float32)

    def test_three_known_parcels_and_background_rejection(self):
        for y in (-.11, 0, .11):
            u = round(320-y*550/.855)
            v = round(240-(.56-.4)*550/.855)
            self.depth[v-25:v+26, u-15:u+16] = .855
        found = detect(self.depth, self.k, self.rotation, self.translation)
        self.assertEqual(len(found), 3)
        for measured, expected_y in zip(found, (.11, 0, -.11)):
            self.assertLess(abs(measured[0]-.56), .003)
            self.assertLess(abs(measured[1]-expected_y), .003)
            self.assertLess(abs(measured[2]-.32), .001)
        self.depth[:] = .905
        self.assertEqual(detect(self.depth, self.k, self.rotation, self.translation), [])

    def test_invalid_depth_and_wrong_size_do_not_become_parcels(self):
        self.depth[100:160, 300:340] = np.nan
        self.depth[180:185, 310:315] = .855
        self.assertEqual(detect(self.depth, self.k, self.rotation, self.translation), [])

    def test_full_footprint_required(self):
        for slot in SLOTS:
            self.assertTrue(inside_tote(*slot))
        self.assertFalse(inside_tote(.43, -.32))
        self.assertFalse(inside_tote(.32, -.21))

    def test_world_and_parcel_assets_have_physical_support(self):
        root = ET.parse(ROOT / 'src/twin_action_demo/worlds/industrial_parcel_cell.sdf')
        belt = root.find("world/model[@name='pick_belt']")
        self.assertEqual(belt.findtext('plugin/velocity_topic'), '/parcel_cell/belt_speed')
        self.assertIsNotNone(root.find("world/model[@name='collection_tote_blue']/link/collision[@name='floor']"))
        model = ET.parse(ROOT / 'src/twin_action_demo/models/parcel_box.sdf')
        self.assertEqual(model.findtext('model/static'), 'false')
        self.assertEqual(model.findtext('model/link/collision/geometry/box/size'), '0.08 0.05 0.05')

    def test_tote_station_legs_clear_the_pedestal(self):
        root = ET.parse(ROOT / 'src/twin_action_demo/worlds/industrial_parcel_cell.sdf')
        def box(collision):
            centre = np.array(list(map(float, collision.findtext('pose').split()))[:3])
            half = np.array(list(map(float, collision.findtext('geometry/box/size').split()))) / 2
            return centre-half, centre+half
        pedestal = [box(c) for c in root.findall("world/model[@name='robot_pedestal']/link/collision")]
        legs = [box(c) for c in root.findall("world/model[@name='tote_station']/link/collision")
                if c.get('name').startswith('leg_')]
        self.assertEqual(len(legs), 4)
        for lo, hi in legs:
            for base_lo, base_hi in pedestal:
                overlap = np.minimum(hi, base_hi) - np.maximum(lo, base_lo)
                self.assertFalse(np.all(overlap > .0001), 'Station leg intersects the fixed plinth')


if __name__ == '__main__':
    unittest.main()
