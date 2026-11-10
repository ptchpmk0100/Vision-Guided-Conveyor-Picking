"""Sensor-only grasp confirmation and retention; all lengths are per-finger metres."""
from collections import deque
from dataclasses import dataclass
import math
import re
from twin_moveit_scripts.parcel_layout import active_batch


def parcel_from_collision(name,allowed=None):
    match = re.search(r'(?:^|::)(parcel_[1-9][0-9]*)(?:::|$)', name)
    allowed=active_batch().names if allowed is None else allowed
    return match.group(1) if match and match.group(1) in allowed else None


@dataclass(frozen=True)
class GripSample:
    stamp_ns: int
    positions: tuple
    desired: float
    left: frozenset
    right: frozenset
    joint_age: float = 0.
    contact_age: float = 0.


class GraspEvaluator:
    def __init__(self, persistence=.10, loss_hold=.05,allowed_objects=None):
        self.allowed_objects=frozenset(active_batch().names if allowed_objects is None else allowed_objects)
        self.persistence = persistence
        self.loss_hold = loss_hold
        self.history = deque()
        self.last_stamp = -1
        self.confirmed = False
        self.object_id = None
        self.bad_since = None
        self.reason = 'awaiting feedback'

    def update(self, sample):
        values = (*sample.positions, sample.desired, sample.joint_age, sample.contact_age)
        trustworthy = (len(sample.positions) == 2 and all(math.isfinite(v) for v in values)
                       and -.02 <= sample.joint_age <= .10 and -.02 <= sample.contact_age <= .10)
        if not trustworthy:
            self.confirmed=False
            self.history.clear()
            self.reason='stale/invalid feedback'
            return False,self.reason
        if sample.stamp_ns <= self.last_stamp:
            return self.confirmed, 'duplicate feedback'
        self.last_stamp = sample.stamp_ns
        now = sample.stamp_ns/1e9
        pair = sample.left & sample.right
        identity = next(iter(pair)) if len(pair) == 1 else None
        if (identity is None or identity not in self.allowed_objects
                or sample.left != sample.right or len(sample.left) != 1):
            identity = None
        width = sum(sample.positions)/2 if len(sample.positions) == 2 else float('nan')
        valid = (trustworthy and all(-.0005 <= q <= .0405 for q in sample.positions)
                 and .019 <= width <= .031 and width-sample.desired > .0015
                 and identity is not None
                 and (self.object_id is None or identity == self.object_id))
        if not valid:
            self.history.clear()
            self.reason = ('stale/invalid feedback' if not trustworthy else
                           'no bilateral parcel contact or seated compression')
            if not trustworthy or self.bad_since is None:
                self.bad_since = now
            if self.confirmed and (not trustworthy or now-self.bad_since >= self.loss_hold-1e-9):
                self.confirmed = False
                self.reason = 'grasp lost: '+self.reason
            return self.confirmed, self.reason
        self.bad_since = None
        self.history.append((now, width, identity))
        while len(self.history) > 1 and now-self.history[1][0] >= self.persistence:
            self.history.popleft()
        if now-self.history[0][0] >= self.persistence-1e-9:
            widths = [r[1] for r in self.history]
            stable = (max(widths)-min(widths) <= .0005 and
                      abs(widths[-1]-widths[0])/max(now-self.history[0][0], .001) <= .005)
            if stable and all(r[2] == identity for r in self.history):
                self.confirmed = True
                self.object_id = identity
                self.reason = 'bilateral contact and stable seated gap'
        return self.confirmed, self.reason


def close_command(elapsed):
    return .04-.02*max(0., min(1., elapsed))
