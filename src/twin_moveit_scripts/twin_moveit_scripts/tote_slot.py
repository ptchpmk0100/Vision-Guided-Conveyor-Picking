"""Pure stability rule for confirming a released parcel in its tote slot (no ROS)."""


class SlotConfirmation:
    """A slot is confirmed after `window` seconds of CONTINUOUS valid observations.

    A gap longer than `max_gap` between valid samples (arm blocking the update loop,
    dropped frames) restarts the window, and an invalid sample resets it. After
    `timeout` simulated seconds without confirmation the release is a fault.
    """

    def __init__(self, x, y, started, window=.6, max_gap=.3, timeout=12.):
        self.x, self.y = x, y
        self.started = started
        self.window, self.max_gap, self.timeout = window, max_gap, timeout
        self.since = self.last = None

    def update(self, valid, stamp, now):
        """Return True once confirmed. Raise RuntimeError on timeout."""
        if valid and stamp != self.last:
            if self.last is None or stamp-self.last > self.max_gap or self.since is None:
                self.since = stamp
            self.last = stamp
            if stamp-self.since >= self.window:
                return True
        elif not valid:
            self.since = self.last = None
        if now-self.started > self.timeout:
            raise RuntimeError('Released parcel did not settle in its tote slot in depth observations')
        return False
