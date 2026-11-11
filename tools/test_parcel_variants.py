#!/usr/bin/env python3
"""Parcel appearance variants: identical physics, bounded decals, deterministic, known condition."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from twin_moveit_scripts.parcel_variants import (HALF_X, HALF_Y, HALF_Z, MAX_RISE, parse_damaged,
                                                  physics_signature, variant_sdf, write_variants)

BASE = (Path(__file__).resolve().parents[1] / 'src/twin_action_demo/models/parcel_box.sdf').read_text()


def _extent(visual):
    """(max |x|, max |y|, top z) of one box/cylinder decal including yaw."""
    import math
    x, y, z, _, _, yaw = map(float, visual.find('pose').text.split())
    if visual.find('geometry/box') is not None:
        sx, sy, sz = map(float, visual.find('geometry/box/size').text.split())
    else:
        r = float(visual.find('geometry/cylinder/radius').text)
        sx = sy = 2 * r
        sz = float(visual.find('geometry/cylinder/length').text)
    hx = (abs(math.cos(yaw)) * sx + abs(math.sin(yaw)) * sy) / 2
    hy = (abs(math.sin(yaw)) * sx + abs(math.cos(yaw)) * sy) / 2
    return abs(x) + hx, abs(y) + hy, z + sz / 2


class VariantTest(unittest.TestCase):
    def test_physics_identical_for_every_variant(self):
        ref = physics_signature(BASE)
        for seed in ('a', 'b', '7'):
            for index in range(1, 11):
                for damaged in (False, True):
                    text, _ = variant_sdf(BASE, seed, index, damaged)
                    self.assertEqual(physics_signature(text), ref)

    def test_damage_decals_inside_top_face_and_thin(self):
        seen = set()
        for seed in range(40):
            text, info = variant_sdf(BASE, str(seed), 3, True)
            link = ET.fromstring(text).find('model/link')
            self.assertGreaterEqual(len(info['damage']), 1)
            for visual in link.findall('visual'):
                name = visual.get('name')
                if name.startswith(('damage_', 'benign_')):
                    ex, ey, top = _extent(visual)
                    self.assertLessEqual(ex, HALF_X + 1e-6, name)
                    self.assertLessEqual(ey, HALF_Y + 1e-6, name)
                    self.assertLessEqual(top - HALF_Z, MAX_RISE, name)
            seen.update(d['kind'] for d in info['damage'])
        self.assertEqual(seen, {'stain', 'tear', 'crush'})

    def test_intact_has_no_damage_visual_and_label_stays_on_top(self):
        for seed in range(40):
            text, info = variant_sdf(BASE, str(seed), 1, False)
            link = ET.fromstring(text).find('model/link')
            self.assertEqual(info['condition'], 'intact')
            self.assertFalse([v for v in link.findall('visual') if v.get('name').startswith('damage_')])
            for visual in link.findall('visual'):
                if visual.get('name') == 'label_visual' or visual.get('name').startswith('barcode_'):
                    ex, ey, _ = _extent(visual)
                    self.assertLessEqual(ex, HALF_X)
                    self.assertLessEqual(ey, HALF_Y)

    def test_deterministic_and_seed_sensitive(self):
        a = variant_sdf(BASE, 'x', 2, True)
        self.assertEqual(a, variant_sdf(BASE, 'x', 2, True))
        self.assertNotEqual(a[0], variant_sdf(BASE, 'y', 2, True)[0])

    def test_write_and_parse(self):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            files, manifest = write_variants(tmp, Path(__file__).resolve().parents[1] /
                                             'src/twin_action_demo/models/parcel_box.sdf', 10, {2, 5}, 's')
            self.assertEqual(len(files), 10)
            self.assertEqual([m['condition'] for m in manifest].count('damaged'), 2)
            self.assertEqual(manifest[1]['condition'], 'damaged')
            self.assertTrue(all(Path(p).exists() for p in files.values()))
        self.assertEqual(parse_damaged('2, 5', 10), {2, 5})
        self.assertEqual(parse_damaged('', 10), set())
        with self.assertRaises(ValueError):
            parse_damaged('11', 10)


if __name__ == '__main__':
    unittest.main()
