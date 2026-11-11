"""Deterministic parcel appearance variants (intact / damaged) with IDENTICAL physics.

Only <visual> elements differ from models/parcel_box.sdf: the collision box, mass and inertia are copied
untouched, and every added visual lies inside the carton top face and rises at most MAX_RISE above it
(the depth top-plane tolerance is 6 mm), so grasping and the depth detector cannot tell the variants apart.
Condition is known by construction: the caller decides which parcels are damaged.
"""
import copy
import math
import random
import xml.etree.ElementTree as ET

HALF_X, HALF_Y, HALF_Z = 0.040, 0.025, 0.025
MAX_RISE = 0.0008            # highest point of any added visual above the carton top (m)
CLASSES = ('intact', 'damaged')
DAMAGE_KINDS = ('stain', 'tear', 'crush')
_LAYER = HALF_Z + 0.0004     # decal centre height; decals are 0.2 mm thick


def _num(values):
    return ' '.join(f'{v:.6g}' for v in values)


def _visual(name, pose, geometry, colour):
    v = ET.Element('visual', name=name)
    ET.SubElement(v, 'pose').text = _num(pose)
    g = ET.SubElement(v, 'geometry')
    if geometry[0] == 'box':
        ET.SubElement(ET.SubElement(g, 'box'), 'size').text = _num(geometry[1])
    else:
        c = ET.SubElement(g, 'cylinder')
        ET.SubElement(c, 'radius').text = f'{geometry[1]:.6g}'
        ET.SubElement(c, 'length').text = f'{geometry[2]:.6g}'
    m = ET.SubElement(v, 'material')
    rgba = _num((*colour, 1))
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(m, tag).text = rgba
    ET.SubElement(m, 'specular').text = '0.05 0.05 0.05 1'
    return v


def _top_point(rng, margin):
    return (rng.uniform(-HALF_X + margin, HALF_X - margin), rng.uniform(-HALF_Y + margin, HALF_Y - margin))


def _stain(rng, link, n):
    cx, cy = _top_point(rng, 0.018)
    base = rng.uniform(0.16, 0.34)
    colour = (base, base * 0.72, base * 0.45)
    count = rng.randint(2, 4)
    for i in range(count):
        r = rng.uniform(0.010, 0.017)
        x = min(HALF_X - r, max(-HALF_X + r, cx + rng.uniform(-0.008, 0.008)))
        y = min(HALF_Y - r, max(-HALF_Y + r, cy + rng.uniform(-0.006, 0.006)))
        link.append(_visual(f'damage_stain_{n}_{i}', (x, y, _LAYER + 0.00005 * i, 0, 0, 0),
                            ('cylinder', r, 0.0002), colour))
    return {'kind': 'stain', 'centre': [round(cx, 4), round(cy, 4)], 'blobs': count}


def _tear(rng, link, n):
    sx, sy = rng.choice((-1, 1)), rng.choice((-1, 1))
    length, width = rng.uniform(0.032, 0.046), rng.uniform(0.006, 0.009)
    yaw = rng.uniform(-0.5, 0.5) + (math.pi / 2 if rng.random() < 0.4 else 0)
    flap = (length * 0.9, width * 1.9)
    off = width * 1.4

    def half(sx_, sy_):
        return ((abs(math.cos(yaw)) * sx_ + abs(math.sin(yaw)) * sy_) / 2,
                (abs(math.sin(yaw)) * sx_ + abs(math.cos(yaw)) * sy_) / 2)
    cx = sx * rng.uniform(0, HALF_X)
    cy = sy * rng.uniform(0, HALF_Y)
    for _ in range(60):          # pull slit + flap together until both lie inside the top face
        fx, fy = cx - math.sin(yaw) * off, cy + math.cos(yaw) * off
        (hx, hy), (fhx, fhy) = half(length, width), half(*flap)
        if (abs(cx) + hx <= HALF_X - 0.003 and abs(cy) + hy <= HALF_Y - 0.003 and
                abs(fx) + fhx <= HALF_X - 0.003 and abs(fy) + fhy <= HALF_Y - 0.003):
            break
        cx, cy = cx * 0.9, cy * 0.9
    fyaw = yaw
    link.append(_visual(f'damage_tear_flap_{n}', (fx, fy, _LAYER, 0, 0, fyaw),
                        ('box', (flap[0], flap[1], 0.0002)), (0.90, 0.80, 0.62)))
    link.append(_visual(f'damage_tear_slit_{n}', (cx, cy, _LAYER + 0.0001, 0, 0, yaw),
                        ('box', (length, width, 0.0002)), (0.07, 0.05, 0.03)))
    return {'kind': 'tear', 'centre': [round(cx, 4), round(cy, 4)], 'length': round(length, 4)}


def _crush(rng, link, n):
    sx, sy = rng.choice((-1, 1)), rng.choice((-1, 1))
    size = rng.uniform(0.016, 0.022)
    reach = size * math.sqrt(2) / 2 + 0.001      # half-diagonal of the 45-degree square
    cx, cy = sx * (HALF_X - reach), sy * (HALF_Y - reach)
    link.append(_visual(f'damage_crush_corner_{n}', (cx, cy, _LAYER, 0, 0, math.pi / 4),
                        ('box', (size, size, 0.0002)), (0.20, 0.14, 0.08)))
    # crease lines radiating from the crushed corner, darker than the carton
    for i, a in enumerate((rng.uniform(0.4, 0.7), rng.uniform(1.0, 1.3))):
        ln = rng.uniform(0.022, 0.034)
        ang = math.atan2(-sy, -sx) + (a - 0.85)
        x = min(HALF_X - 0.003, max(-HALF_X + 0.003, cx + math.cos(ang) * ln / 2))
        y = min(HALF_Y - 0.003, max(-HALF_Y + 0.003, cy + math.sin(ang) * ln / 2))
        link.append(_visual(f'damage_crush_crease_{n}_{i}', (x, y, _LAYER + 0.00005, 0, 0, ang),
                            ('box', (ln, 0.0035, 0.0002)), (0.18, 0.12, 0.07)))
    return {'kind': 'crush', 'corner': [sx, sy], 'size': round(size, 4)}


_DAMAGE = {'stain': _stain, 'tear': _tear, 'crush': _crush}


def _benign(rng, link):
    """Harmless extras on BOTH classes so 'extra marks' alone does not mean damaged."""
    notes = []
    if rng.random() < 0.5:
        x, y = _top_point(rng, 0.012)
        colour = rng.choice(((0.78, 0.12, 0.10), (0.10, 0.35, 0.75), (0.92, 0.80, 0.15)))
        link.append(_visual('benign_sticker', (x, y, _LAYER, 0, 0, rng.uniform(-0.3, 0.3)),
                            ('box', (0.011, 0.011, 0.0002)), colour))
        notes.append('sticker')
    return notes


def variant_names(count, damaged):
    return [(f'parcel_{i}', i in damaged) for i in range(1, count + 1)]


def variant_sdf(base_text, seed, index, damaged):
    """Return (sdf_text, info) for parcel `index` (1-based)."""
    rng = random.Random(f'{seed}:{index}')
    root = ET.fromstring(base_text)
    link = root.find('model/link')
    info = {'index': index, 'name': f'parcel_{index}', 'condition': 'damaged' if damaged else 'intact',
            'damage': [], 'benign': []}
    carton = link.find("visual[@name='carton_visual']")
    shade = rng.uniform(0.88, 1.08)
    for tag in ('ambient', 'diffuse'):
        r, g, b, a = map(float, carton.find(f'material/{tag}').text.split())
        carton.find(f'material/{tag}').text = _num((min(1, r * shade), min(1, g * shade), min(1, b * shade), a))
    info['shade'] = round(shade, 3)
    # move the shipping label and its barcode strips together
    nx, ny = rng.uniform(-0.022, 0.022), rng.uniform(-0.008, 0.008)
    dx, dy = nx - 0.02, ny - 0.005
    for visual in link.findall('visual'):
        if visual.get('name') == 'label_visual' or visual.get('name', '').startswith('barcode_'):
            pose = list(map(float, visual.find('pose').text.split()))
            pose[0] += dx
            pose[1] += dy
            visual.find('pose').text = _num(pose)
    info['benign'] = _benign(rng, link)
    if damaged:
        kinds = [rng.choice(DAMAGE_KINDS)]
        if rng.random() < 0.3:
            kinds.append(rng.choice([k for k in DAMAGE_KINDS if k != kinds[0]]))
        for n, kind in enumerate(kinds):
            info['damage'].append(_DAMAGE[kind](rng, link, n))
    return ET.tostring(root, encoding='unicode'), info


def write_variants(directory, base_path, count, damaged, seed):
    """Write parcel_<i>.sdf for 1..count; return ({name: path}, manifest list)."""
    from pathlib import Path
    base = Path(base_path).read_text()
    out, manifest = {}, []
    for i in range(1, count + 1):
        text, info = variant_sdf(base, seed, i, i in damaged)
        path = Path(directory) / f'parcel_variant_{i}.sdf'
        path.write_text(text)
        out[f'parcel_{i}'] = str(path)
        manifest.append(info)
    return out, manifest


def parse_damaged(text, count):
    """'2,5,9' -> {2,5,9}; validates the indices."""
    items = {int(t) for t in str(text).replace(' ', '').split(',') if t}
    bad = sorted(i for i in items if not 1 <= i <= count)
    if bad:
        raise ValueError(f'damaged parcel indices {bad} outside 1..{count}')
    return items


def physics_signature(sdf_text):
    """Serialised <inertial> and <collision> blocks; equal for every variant."""
    link = ET.fromstring(sdf_text).find('model/link')
    return ET.tostring(link.find('inertial')) + b''.join(
        ET.tostring(c) for c in link.findall('collision'))
