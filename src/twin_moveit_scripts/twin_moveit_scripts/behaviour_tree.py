#!/usr/bin/env python3
"""A tiny dependency-free behaviour-tree core.

The public surface is deliberately small: Status, Behaviour, Sequence, Selector,
Retry, and Leaf.  A future py_trees migration only needs adapters for those concepts.

Offline self-test:
  python3 src/twin_moveit_scripts/twin_moveit_scripts/behaviour_tree.py --self-test
"""
import enum
import sys


class Status(enum.Enum):
    SUCCESS = enum.auto()
    FAILURE = enum.auto()
    RUNNING = enum.auto()


class Behaviour:
    """Base behaviour with one public tick and a reset hook."""

    def __init__(self, name, logger=None):
        self.name = str(name)
        if logger is None:
            self._log = print
        elif callable(logger):
            self._log = logger
        elif hasattr(logger, "info"):
            self._log = logger.info
        else:
            raise TypeError("logger must be callable or expose info(message)")

    def tick(self):
        return self._tick(0)

    def _tick(self, depth):
        status = self._update(depth)
        if not isinstance(status, Status):
            raise TypeError(
                f"{self.name} returned {status!r}; expected a Status"
            )
        self._log(f"[{'  ' * depth}{self.name} -> {status.name}]")
        return status

    def _update(self, depth):
        raise NotImplementedError

    def reset(self):
        """Reset any RUNNING state; leaves are stateless by default."""


class Sequence(Behaviour):
    """Memory sequence: stop on FAILURE/RUNNING, succeed when all succeed."""

    def __init__(self, name, children, logger=None):
        super().__init__(name, logger)
        self.children = list(children)
        self._index = 0

    def _update(self, depth):
        while self._index < len(self.children):
            status = self.children[self._index]._tick(depth + 1)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.FAILURE:
                self.reset()
                return Status.FAILURE
            self._index += 1
        self.reset()
        return Status.SUCCESS

    def reset(self):
        self._index = 0
        for child in self.children:
            child.reset()


class Selector(Behaviour):
    """Memory selector: first SUCCESS/RUNNING wins; fail if all fail."""

    def __init__(self, name, children, logger=None):
        super().__init__(name, logger)
        self.children = list(children)
        self._index = 0

    def _update(self, depth):
        while self._index < len(self.children):
            status = self.children[self._index]._tick(depth + 1)
            if status is Status.RUNNING:
                return Status.RUNNING
            if status is Status.SUCCESS:
                self.reset()
                return Status.SUCCESS
            self._index += 1
        self.reset()
        return Status.FAILURE

    def reset(self):
        self._index = 0
        for child in self.children:
            child.reset()


class Retry(Behaviour):
    """Retry a failed child at most ``n`` total attempts."""

    def __init__(self, child, n, name=None, logger=None):
        if int(n) < 1:
            raise ValueError("Retry n must be >= 1")
        self.child = child
        self.n = int(n)
        super().__init__(name or f"Retry({child.name},{self.n})", logger)
        self._failures = 0

    def _update(self, depth):
        status = self.child._tick(depth + 1)
        if status is Status.RUNNING:
            return Status.RUNNING
        if status is Status.SUCCESS:
            self.reset()
            return Status.SUCCESS

        self._failures += 1
        self.child.reset()
        if self._failures >= self.n:
            self.reset()
            return Status.FAILURE
        return Status.RUNNING

    def reset(self):
        self._failures = 0
        self.child.reset()


class Leaf(Behaviour):
    """Wrap a no-argument callable returning Status."""

    def __init__(self, name, fn, logger=None):
        super().__init__(name, logger)
        if not callable(fn):
            raise TypeError("Leaf fn must be callable")
        self.fn = fn

    def _update(self, depth):
        return self.fn()


def _self_test():
    trace = []

    def log(message):
        trace.append(message)
        print(message)

    sequence_calls = []
    sequence = Sequence("SequenceTest", [
        Leaf("SeqSuccess", lambda: sequence_calls.append("success") or Status.SUCCESS,
             log),
        Leaf("SeqFailure", lambda: sequence_calls.append("failure") or Status.FAILURE,
             log),
        Leaf("MustNotRun", lambda: sequence_calls.append("bad") or Status.SUCCESS,
             log),
    ], log)
    assert sequence.tick() is Status.FAILURE
    assert sequence_calls == ["success", "failure"]

    selector_calls = []
    selector = Selector("SelectorTest", [
        Leaf("SelFailure", lambda: selector_calls.append("failure") or Status.FAILURE,
             log),
        Leaf("SelSuccess", lambda: selector_calls.append("success") or Status.SUCCESS,
             log),
        Leaf("MustNotRun", lambda: selector_calls.append("bad") or Status.SUCCESS,
             log),
    ], log)
    assert selector.tick() is Status.SUCCESS
    assert selector_calls == ["failure", "success"]

    retry_calls = []

    def flaky():
        retry_calls.append(len(retry_calls) + 1)
        return Status.SUCCESS if len(retry_calls) == 3 else Status.FAILURE

    retry = Retry(Leaf("Flaky", flaky, log), 3,
                  name="RetrySuccessTest", logger=log)
    assert [retry.tick(), retry.tick(), retry.tick()] == [
        Status.RUNNING, Status.RUNNING, Status.SUCCESS,
    ]
    assert retry_calls == [1, 2, 3]

    failed_calls = []
    retry_fail = Retry(
        Leaf("AlwaysFails",
             lambda: failed_calls.append(1) or Status.FAILURE, log),
        2, name="RetryFailureTest", logger=log,
    )
    assert retry_fail.tick() is Status.RUNNING
    assert retry_fail.tick() is Status.FAILURE
    assert len(failed_calls) == 2

    print("BT SELF-TEST: Sequence short-circuit PASS")
    print("BT SELF-TEST: Selector fallback PASS")
    print("BT SELF-TEST: Retry success/failure bounds PASS")
    print(f"BT SELF-TEST: tick log captured {len(trace)} entries")
    print("BT SELF-TEST PASS")


if __name__ == "__main__":
    if sys.argv[1:] not in ([], ["--self-test"]):
        raise SystemExit("usage: behaviour_tree.py [--self-test]")
    _self_test()
