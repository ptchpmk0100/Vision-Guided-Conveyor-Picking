"""Camera-only conveyor association and interception math, independent of ROS."""
from dataclasses import dataclass, field
import math
import statistics


def follow_distance(elapsed, speed, total=4.5, deceleration=.4):
    """Track at belt speed, then smoothly stop after lifting clear of the deck."""
    if not 0 <= elapsed <= total or not 0 < deceleration < total:
        raise ValueError('Invalid following path timing')
    if elapsed <= total-deceleration:
        return speed*elapsed
    u = (elapsed-(total-deceleration))/deceleration
    # Integral of a smoothstep velocity taper: 1 - 3u^2 + 2u^3.
    return speed*(total-deceleration)+speed*deceleration*(u-u**3+.5*u**4)


@dataclass
class Track:
    number: int
    samples: list = field(default_factory=list)

    def estimate(self):
        if len(self.samples) < 5 or self.samples[-1][0]-self.samples[0][0] < .38:
            return None
        # Median pairwise slopes reject depth-pixel quantization without basing a
        # four-second interception on a single short 100 ms displacement.
        velocities = [(b[1][1]-a[1][1])/(b[0]-a[0])
                      for i, a in enumerate(self.samples) for b in self.samples[i+1:]
                      if b[0]-a[0] > .18]
        return statistics.median(velocities) if velocities else None

    def predict(self, stamp, velocity):
        seen, (x, y, z) = self.samples[-1]
        if not 0 <= stamp-seen <= 1.5:
            raise RuntimeError('Cannot intercept a stale camera track')
        return x, y+velocity*(stamp-seen), z


class BeltTracker:
    def __init__(self, speed):
        if not math.isfinite(speed) or not .02 <= speed <= .08:
            raise ValueError('Continuous belt speed must be in [.02, .08] m/s')
        self.speed = speed
        self.tracks = []
        self.next_number = 1
        self.last_stamp = None

    def update(self, stamp, positions):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        available = [t for t in self.tracks if stamp-t.samples[-1][0] <= 1.0]
        updated = []
        # Near the ROI edge a partly clipped carton can satisfy segmentation's
        # minimum size, but its centroid is biased toward the image interior.
        # Fit velocity only once the complete 50 mm footprint clears that edge.
        for xyz in (p for p in positions if -.365 <= p[1] <= .145):
            matches = [(math.dist(t.predict(stamp, self.speed), xyz), t) for t in available]
            distance, track = min(matches, key=lambda v: v[0], default=(float('inf'), None))
            if distance > .025:
                track = Track(self.next_number)
                self.next_number += 1
            else:
                available.remove(track)
            track.samples.append((stamp, tuple(xyz)))
            # Retain ~1 sim-s of history so a single detector jitter gap (up to ~0.8
            # sim-s under GUI load) does not wipe an accumulating track and force a
            # slow rebuild that misses the narrow interception window. Velocity is still
            # a median of pairwise slopes (longer baseline = more accurate), and
            # predict() uses only the latest sample, so grasp targeting is unchanged.
            track.samples = [s for s in track.samples if stamp-s[0] <= 1.0]
            updated.append(track)
        self.tracks = updated+available

    def nearest(self, now, hand_xyz, window=(-.160, -.075)):
        candidates = []
        for track in self.tracks:
            if now-track.samples[-1][0] > .25:
                continue
            velocity = track.estimate()
            if velocity is None:
                continue
            # Camera displacement validates the known constant conveyor demand.
            # Use that demand for following; pixel quantization in a short camera
            # track otherwise accumulates into centimetres of grip drift.
            xyz = track.predict(now, self.speed)
            if xyz[1] < window[0]:
                continue  # build the track upstream, then wait for reachable interception
            if xyz[1] > window[1]:
                raise RuntimeError('Parcel passed the continuous intercept window; reset the batch')
            if abs(velocity-self.speed) > .010:
                # An initial camera fit can be uncertain. Keep observing while
                # the parcel is still interceptable; never pick on that fit.
                continue
            candidates.append((math.dist(xyz, hand_xyz), track, self.speed))
        if not candidates:
            return None
        _, track, velocity = min(candidates, key=lambda v: v[0])
        return track, velocity
