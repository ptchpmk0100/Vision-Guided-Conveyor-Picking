"""Measured per-joint arrival predicate, simulation shadow/replay only.

All acquisition stamps and `now` must share one caller-declared clock domain.
An action result must be explicitly associated with this command. This module
does not command motion or replace the existing settle/arrival decision.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ArrivalProfile:
    position_tolerance: tuple
    velocity_tolerance: tuple
    continuous: tuple
    max_age: float
    dwell: float
    timeout: float

    def __post_init__(self):
        n = len(self.continuous)
        if not n or len(self.position_tolerance) != n or len(self.velocity_tolerance) != n:
            raise ValueError('Invalid per-joint profile shape')
        if any(not isinstance(c, bool) for c in self.continuous):
            raise ValueError('Continuous flags must be booleans')
        values = (*self.position_tolerance, *self.velocity_tolerance,
                  self.max_age, self.dwell, self.timeout)
        if any(not isinstance(v, (int, float)) or isinstance(v, bool)
               or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Arrival tolerances and durations must be finite and positive')


class ArrivalGate:
    def __init__(self, profile):
        self.profile = profile
        self.command_id = None
        self.terminal = False

    def start(self, command_id, target, now):
        if not command_id or len(target) != len(self.profile.continuous) or not all(
                math.isfinite(v) for v in (*target, now)):
            raise ValueError('Invalid command/target/clock')
        self.command_id = command_id
        self.target = tuple(target)
        self.started = self.last_now = now
        self.last_stamp = self.since = None
        self.terminal = False

    def update(self, *, now, stamp, positions, velocities,
               result_command_id=None, controller_succeeded=False, fault=False):
        if self.command_id is None:
            raise RuntimeError('No active command')
        if self.terminal:
            raise RuntimeError('Start a new command after a terminal verdict')
        if not math.isfinite(now) or now < self.last_now:
            self.terminal = True
            return 'CLOCK_RESET'
        self.last_now = now
        if fault or now-self.started >= self.profile.timeout:
            self.terminal = True
            return 'FAULT' if fault else 'TIMEOUT'
        def wait(reason):
            self.since = None
            return reason
        if stamp is None or not math.isfinite(stamp) or stamp > now or now-stamp > self.profile.max_age:
            return wait('STATE_STALE')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return wait('SAMPLE_NOT_INCREASING')
        if self.last_stamp is not None and stamp-self.last_stamp > self.profile.max_age:
            self.since = None
        self.last_stamp = stamp
        n = len(self.target)
        if positions is None or velocities is None or len(positions) != n or len(velocities) != n:
            return wait('STATE_MISSING')
        if not all(math.isfinite(v) for v in (*positions, *velocities)):
            return wait('STATE_NONFINITE')
        if result_command_id != self.command_id or not controller_succeeded:
            return wait('CONTROLLER_UNCONFIRMED')
        for q, target, velocity, ep, ev, continuous in zip(positions, self.target, velocities,
                self.profile.position_tolerance, self.profile.velocity_tolerance, self.profile.continuous):
            error = q-target
            if continuous:
                error = math.remainder(error, 2*math.pi)
            if abs(error) > ep or abs(velocity) > ev:
                return wait('NOT_SETTLED')
        if self.since is None:
            self.since = stamp
        if stamp-self.since >= self.profile.dwell:
            self.terminal = True
            return 'ARRIVED'
        return 'DWELLING'
