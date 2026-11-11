#!/usr/bin/env python3
"""Condition routing: vote rule, go/wait/skip decision, router, tracker votes/exclude, bin world, geometry clearance."""
from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET

from twin_moveit_scripts import parcel_reject as pr
from twin_moveit_scripts.parcel_batch_world import batch_world
from twin_moveit_scripts.parcel_layout import PARCEL_SIZE, PARCEL_Z, batch_layout
from twin_moveit_scripts.parcel_tracking import BeltTracker

WORLD = Path(__file__).resolve().parents[1] / 'src/twin_action_demo/worlds/industrial_parcel_cell_continuous.sdf'


class VerdictTest(unittest.TestCase):
    def test_pending_until_enough_votes(self):
        self.assertEqual(pr.verdict([0.99] * (pr.MIN_VOTES - 1))[0], 'pending')
        self.assertEqual(pr.verdict([0.99] * pr.MIN_VOTES)[0], 'damaged')

    def test_thresholds_and_unknown_band(self):
        self.assertEqual(pr.verdict([0.02] * 6)[0], 'intact')
        self.assertEqual(pr.verdict([0.5] * 6)[0], 'unknown')
        self.assertEqual(pr.verdict([0.8] * 6)[0], 'damaged')
        self.assertEqual(pr.verdict([0.3] * 6)[0], 'unknown')

    def test_recent_window_only(self):
        votes = [0.0] * 20 + [1.0] * pr.WINDOW_VOTES
        self.assertEqual(pr.verdict(votes)[0], 'damaged')

    def test_one_bad_frame_does_not_flip_a_verdict(self):
        self.assertEqual(pr.verdict([0.98] * 7 + [0.02])[0], 'damaged')


class ActionRouterTest(unittest.TestCase):
    def test_actions(self):
        self.assertEqual(pr.next_action('damaged', -0.3), 'go')
        self.assertEqual(pr.next_action('intact', -0.1), 'go')
        self.assertEqual(pr.next_action('pending', -0.2), 'wait')
        self.assertEqual(pr.next_action('pending', -0.05), 'skip')     # no votes in time: never guess
        self.assertEqual(pr.next_action('pending', None), 'skip')
        self.assertEqual(pr.next_action('unknown', -0.3), 'skip')

    def test_router_slots_and_limits(self):
        slots = batch_layout(10).slots
        router = pr.Router(slots)
        self.assertEqual(router.assign('intact'), ('tote', 0, slots[0]))
        self.assertEqual(router.assign('damaged'), ('reject', 0, pr.BIN_SLOTS[0]))
        self.assertEqual(router.assign('intact'), ('tote', 1, slots[1]))
        with self.assertRaises(ValueError):
            router.assign('unknown')
        for _ in range(len(pr.BIN_SLOTS) - 1):
            router.assign('damaged')
        with self.assertRaises(RuntimeError):
            router.assign('damaged')
        tote = pr.Router(slots[:1])
        tote.assign('intact')
        with self.assertRaises(RuntimeError):
            tote.assign('intact')


class TrackerVotesTest(unittest.TestCase):
    def feed(self, tracker, n=8, ys=(-0.30, -0.10)):
        for i in range(n):
            t = i * 0.1
            tracker.update(t, [(0.56, y + 0.045 * t, 0.32) for y in ys])

    def test_votes_attach_to_the_right_track_once_per_stamp(self):
        tr = BeltTracker(0.045)
        self.feed(tr)
        a = min(tr.tracks, key=lambda t: t.samples[-1][1][1])    # the track nearer y = -0.30
        b = next(t for t in tr.tracks if t is not a)
        t = 0.7
        near = (0.56, -0.30 + 0.045 * t, 0.32)
        far = (0.56, -0.10 + 0.045 * t, 0.32)
        self.assertEqual(tr.add_condition(t, [(near, 0.95), (far, 0.05)]), 2)
        self.assertEqual(a.votes, [0.95])
        self.assertEqual(b.votes, [0.05])
        self.assertEqual(tr.add_condition(t, [(near, 0.95)]), 0)       # same stamp: no double vote
        self.assertEqual(tr.add_condition(t + 0.1, [((0.56, 0.9, 0.32), 0.9)]), 0)   # nothing near
        self.assertIs(tr.track_by_number(a.number), a)

    def test_late_condition_message_does_not_raise(self):
        tr = BeltTracker(0.045)
        self.feed(tr)
        tr.add_condition(0.2, [((0.56, -0.29, 0.32), 0.9)])   # older than the newest sample

    def test_excluded_track_may_pass_the_window_without_error(self):
        tr = BeltTracker(0.045)
        for i in range(12):
            t = i * 0.1
            tr.update(t, [(0.56, -0.08 + 0.045 * t, 0.32)])
        track = tr.tracks[0]
        with self.assertRaises(RuntimeError):
            tr.nearest(1.1, (0.56, 0.06, 0.52), window=(-0.25, -0.075 - 0.2))
        self.assertIsNone(tr.nearest(1.1, (0.56, 0.06, 0.52), window=(-0.25, -0.075 - 0.2), exclude={track.number}))


class BinGeometryTest(unittest.TestCase):
    def world(self, reject):
        return batch_world(WORLD, batch_layout(10), reject_bin=reject).find('world')

    def test_default_world_has_no_bin(self):
        names = [m.get('name') for m in self.world(False).findall('model')]
        self.assertNotIn('reject_bin_red', names)
        self.assertNotIn('reject_station', names)

    def test_bin_world_adds_bin_and_stand_only(self):
        a = [m.get('name') for m in self.world(False).findall('model')]
        b = [m.get('name') for m in self.world(True).findall('model')]
        self.assertEqual([n for n in b if n not in a], ['reject_bin_red', 'reject_station'])

    def test_slots_fit_and_match_tote_release_geometry(self):
        for x, y in pr.BIN_SLOTS:
            self.assertTrue(pr.inside_bin(x, y), (x, y))
        xs = sorted({x for x, _ in pr.BIN_SLOTS})
        ys = sorted({y for _, y in pr.BIN_SLOTS})
        self.assertGreaterEqual(min(b - a for a, b in zip(xs, xs[1:])), 0.10 - 1e-9)    # tote pitch
        self.assertGreaterEqual(min(b - a for a, b in zip(ys, ys[1:])), 0.10 - 1e-9)
        self.assertFalse(pr.inside_bin(pr.BIN_XY[0], pr.BIN_XY[1] + 0.2))

    def boxes(self, world):
        out = []
        for m in world.findall('model'):
            for c in m.findall('link/collision'):
                b = c.find('geometry/box/size')
                if b is None:
                    continue
                p = list(map(float, (c.findtext('pose') or '0 0 0 0 0 0').split()))
                mp = list(map(float, (m.findtext('pose') or '0 0 0 0 0 0').split()))
                d = list(map(float, b.text.split()))
                yaw = mp[5]
                cx = mp[0] + math.cos(yaw) * p[0] - math.sin(yaw) * p[1]
                cy = mp[1] + math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
                hx = (abs(math.cos(yaw)) * d[0] + abs(math.sin(yaw)) * d[1]) / 2
                hy = (abs(math.sin(yaw)) * d[0] + abs(math.cos(yaw)) * d[1]) / 2
                out.append((m.get('name'), cx - hx, cx + hx, cy - hy, cy + hy, mp[2] + p[2] - d[2] / 2, mp[2] + p[2] + d[2] / 2))
        return out

    def test_bin_does_not_overlap_other_solids(self):
        boxes = self.boxes(self.world(True))
        mine = [b for b in boxes if b[0] in ('reject_bin_red', 'reject_station')]
        others = [b for b in boxes if b[0] not in ('reject_bin_red', 'reject_station', 'ground_plane')]
        for a in mine:
            for o in others:
                overlap = a[1] < o[2] and o[1] < a[2] and a[3] < o[4] and o[3] < a[4] and a[5] < o[6] and o[5] < a[6]
                self.assertFalse(overlap, (a[0], o[0]))

    def test_bin_floor_is_at_tote_floor_height(self):
        floor = {m.get('name'): m.find("link/collision[@name='floor']") for m in self.world(True).findall('model')
                 if m.get('name') in ('collection_tote_blue', 'reject_bin_red')}
        z = {k: float(v.findtext('pose').split()[2]) for k, v in floor.items()}
        self.assertAlmostEqual(z['reject_bin_red'], z['collection_tote_blue'])
        self.assertAlmostEqual(z['reject_bin_red'] + 0.005 + PARCEL_SIZE[2] / 2, PARCEL_Z, places=3)


if __name__ == '__main__':
    unittest.main()
