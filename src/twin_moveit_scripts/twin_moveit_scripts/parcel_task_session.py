"""Prevent overlapping parcel controllers in the same user's ROS domain."""
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import tempfile


@contextmanager
def task_session(domain=None, directory=None):
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0') if domain is None else domain)
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID must be between 0 and 232')
    root = Path(directory) if directory is not None else (
        Path(tempfile.gettempdir()) / f'twin-parcel-task-locks-{os.getuid()}')
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    if root.is_symlink() or root.stat().st_uid != os.getuid():
        raise RuntimeError('Parcel task lock directory is not owned by the current user')
    fd = os.open(root / f'domain-{domain}.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'r+') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = handle.read(40).strip() or 'unknown'
            raise RuntimeError(
                f'A parcel task is already running in ROS domain {domain} (PID {owner}). '
                'Keep one Terminal 2 task running; wait for it or stop it with Ctrl+C before retrying.'
            ) from None
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
