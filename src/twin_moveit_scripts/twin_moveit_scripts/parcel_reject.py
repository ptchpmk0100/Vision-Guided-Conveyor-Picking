"""Reject bin geometry and the vote rule that decides intact / damaged / unknown for a tracked parcel.

Everything here is OPT-IN (task flag --condition-routing, launch arg reject_bin:=true). With the flag off the cell
behaves exactly as before: the depth detector alone drives the arm and every parcel goes to a tote slot.
"""
import copy
import math
import statistics
import xml.etree.ElementTree as ET

from twin_moveit_scripts.parcel_layout import PARCEL_SIZE

# Bin on the downstream side of the arm, clear of the pedestal (|x|,|y| <= .31), the belt rail (x >= .43) and the
# camera fixture (y >= .77). Same walls, floor height and slot pitch as the tote, so release geometry is the tote's.
BIN_XY = (0.18, 0.50)
BIN_SIZE = (0.34, 0.24)
BIN_SLOTS = tuple((x, y) for x in (0.28, 0.18, 0.08) for y in (0.45, 0.55))   # 6 drops, 0.10 m pitch
BIN_WALL = 0.008
BIN_COLOUR = '0.78 0.20 0.17 1'

# Vote rule. votes = per-detection damaged probabilities attached to one belt track.
MIN_VOTES = 4          # need this many classified frames before any verdict
WINDOW_VOTES = 8       # use the most recent frames
DAMAGED_AT = 0.75      # mean damaged probability >= this -> reject
INTACT_AT = 0.25       # mean damaged probability <= this -> tote
# Anything in between is UNKNOWN: skip and log, never guess.


def verdict(votes, min_votes=MIN_VOTES, hi=DAMAGED_AT, lo=INTACT_AT):
    """-> (state, mean damaged probability or None, n). state: 'pending' | 'damaged' | 'intact' | 'unknown'."""
    n = len(votes)
    if n < min_votes:
        return 'pending', None, n
    mean = statistics.fmean(votes[-WINDOW_VOTES:])
    if mean >= hi:
        return 'damaged', mean, n
    if mean <= lo:
        return 'intact', mean, n
    return 'unknown', mean, n


DECIDE_BY_Y = -0.12    # predicted belt y by which a verdict must exist (intercept window ends at y = -0.075)


def next_action(state, track_y, decide_by=DECIDE_BY_Y):
    """What the task does for the nearest launchable track: 'go' | 'wait' | 'skip'.

    go   - verdict is damaged or intact
    wait - not enough votes yet and the parcel still has room on the belt
    skip - verdict is unknown, or votes never arrived in time (never guess)
    """
    if state in ('damaged', 'intact'):
        return 'go'
    if state == 'pending' and track_y is not None and track_y < decide_by:
        return 'wait'
    return 'skip'


class Router:
    """Hands out the next free tote slot (intact) or bin slot (damaged); raises when a destination is full."""

    def __init__(self, tote_slots, bin_slots=BIN_SLOTS):
        self.tote_slots, self.bin_slots = tuple(tote_slots), tuple(bin_slots)
        self.tote_used = self.bin_used = 0

    def assign(self, state):
        if state == 'damaged':
            if self.bin_used >= len(self.bin_slots):
                raise RuntimeError('Reject bin is full')
            self.bin_used += 1
            return 'reject', self.bin_used - 1, self.bin_slots[self.bin_used - 1]
        if state == 'intact':
            if self.tote_used >= len(self.tote_slots):
                raise RuntimeError('Tote is full')
            self.tote_used += 1
            return 'tote', self.tote_used - 1, self.tote_slots[self.tote_used - 1]
        raise ValueError(f'no destination for verdict {state!r}')


def inside_bin(x, y, yaw=0.0, margin=0.002):
    """Full rotated parcel footprint inside the bin's inner walls."""
    hx = (abs(math.cos(yaw)) * PARCEL_SIZE[0] + abs(math.sin(yaw)) * PARCEL_SIZE[1]) / 2
    hy = (abs(math.sin(yaw)) * PARCEL_SIZE[0] + abs(math.cos(yaw)) * PARCEL_SIZE[1]) / 2
    return (abs(x - BIN_XY[0]) + hx + margin <= BIN_SIZE[0] / 2 - BIN_WALL and
            abs(y - BIN_XY[1]) + hy + margin <= BIN_SIZE[1] / 2 - BIN_WALL)


def add_reject_bin(world, fit_tote):
    """Append reject_bin_red (+ stand) to the <world> element. `fit_tote(link, x, y, w, h)` resizes tote primitives."""
    tote = world.find("model[@name='collection_tote_blue']")
    if tote is None:
        raise ValueError('collection_tote_blue not found in the world')
    if world.find("model[@name='reject_bin_red']") is not None:
        raise ValueError('reject bin already present')
    bin_model = copy.deepcopy(tote)
    bin_model.set('name', 'reject_bin_red')
    link = bin_model.find('link')
    fit_tote(link, BIN_XY[0], BIN_XY[1], BIN_SIZE[0], BIN_SIZE[1])
    for visual in link.findall('visual'):
        for tag in ('ambient', 'diffuse'):
            visual.find(f'material/{tag}').text = BIN_COLOUR
    stand = ET.Element('model', name='reject_station')
    ET.SubElement(stand, 'static').text = 'true'
    ET.SubElement(stand, 'pose').text = '0 0 0 0 0 0'
    sl = ET.SubElement(stand, 'link', name='body')
    size = f'{BIN_SIZE[0] + 0.02:.4f} {BIN_SIZE[1] + 0.02:.4f} 0.635'
    pose = f'{BIN_XY[0]:.4f} {BIN_XY[1]:.4f} -0.0325 0 0 0'
    for tag in ('collision', 'visual'):
        item = ET.SubElement(sl, tag, name='stand' if tag == 'collision' else 'stand_visual')
        ET.SubElement(item, 'pose').text = pose
        ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
        item.find('geometry/box').append(ET.fromstring(f'<size>{size}</size>'))
        if tag == 'visual':
            mat = ET.SubElement(item, 'material')
            for m in ('ambient', 'diffuse'):
                ET.SubElement(mat, m).text = '0.35 0.35 0.38 1'
    index = list(world).index(tote) + 1
    world.insert(index, bin_model)
    world.insert(index + 1, stand)
    return world
