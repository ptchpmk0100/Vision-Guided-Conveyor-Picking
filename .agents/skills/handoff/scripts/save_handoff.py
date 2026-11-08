#!/usr/bin/env python3
"""Save shared agent notes and Git state; no model calls or quota polling."""

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import uuid


def git(root, *args):
    return subprocess.run(
        ["git", "-C", str(root), *args], check=True, capture_output=True,
        text=True, timeout=15,
    ).stdout.rstrip("\n")


def current_id(path):
    if path.is_symlink():
        raise ValueError(f"Refusing to replace symlink: {path}")
    if not path.exists():
        return "none"
    match = re.search(r"^<!-- handoff-id: ([a-f0-9-]+) -->$",
                      path.read_text(encoding="utf-8"), re.MULTILINE)
    if not match:
        raise ValueError("Existing HANDOFF.md has no handoff-id; preserve and reconcile it manually")
    return match.group(1)


def save(root, agent, kind, notes, expected_id):
    root = root.resolve(strict=True)
    if Path(git(root, "rev-parse", "--show-toplevel")).resolve() != root:
        raise ValueError("--root must be the Git repository root")
    if not notes.strip():
        raise ValueError("Notes must not be empty")
    snapshot = root / "HANDOFF.md"
    history = root / "SESSION_LOG.md"
    if history.is_symlink():
        raise ValueError(f"Refusing to append to symlink: {history}")
    # Check before opening history so a stale save never creates a history file.
    if current_id(snapshot) != expected_id:
        raise ValueError("HANDOFF.md changed; reread and reconcile before saving")

    with history.open("a+", encoding="utf-8") as log:
        fcntl.flock(log.fileno(), fcntl.LOCK_EX)
        if current_id(snapshot) != expected_id:
            raise ValueError("HANDOFF.md changed; reread and reconcile before saving")
        entry_id = str(uuid.uuid4())
        timestamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
        branch = git(root, "rev-parse", "--abbrev-ref", "HEAD")
        commit = git(root, "rev-parse", "HEAD")
        status = git(root, "status", "--short", "--untracked-files=all")
        # JSON string literals avoid Git filenames breaking a Markdown code fence.
        status_lines = "\n".join(json.dumps(line) for line in status.splitlines())
        entry = (
            f"<!-- handoff-id: {entry_id} -->\n"
            f"## {timestamp} | {agent} | {kind}\n\n"
            f"- Repository: {json.dumps(str(root))}\n"
            f"- Branch: {json.dumps(branch)}\n"
            f"- Commit: `{commit}`\n"
            "- Git status captured before this save (paths are JSON-escaped):\n\n"
            f"```text\n{status_lines or '(clean)'}\n```\n\n"
            f"{notes.strip()}\n"
        )
        document = (
            "# Current handoff\n\n"
            "Read this snapshot together with the current request and repository guidance.\n"
            "Verify recorded state before resuming. Earlier entries: SESSION_LOG.md.\n\n"
            + entry
        )
        temp_path = None
        try:
            # Prepare a complete snapshot before touching the history.
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=root,
                prefix=".handoff-", suffix=".tmp", delete=False,
            ) as temp:
                temp_path = Path(temp.name)
                temp.write(document)
                temp.flush()
                os.fsync(temp.fileno())
            log.seek(0, os.SEEK_END)
            if log.tell() == 0:
                log.write("# Shared session log\n\nAppend-only history for Claude Code and Codex.\n")
            log.write("\n---\n\n" + entry)
            log.flush()
            os.fsync(log.fileno())
            # If interrupted here, the history retains the entry for recovery.
            os.replace(temp_path, snapshot)
            temp_path = None
            if snapshot.read_text(encoding="utf-8") != document:
                raise OSError("Snapshot read-back did not match the saved entry")
        finally:
            if temp_path is not None:
                temp_path.unlink(missing_ok=True)
        return {"entry_id": entry_id, "agent": agent, "kind": kind,
                "handoff": str(snapshot), "history": str(history)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--agent", choices=("claude", "codex"), required=True)
    parser.add_argument("--kind", choices=("checkpoint", "handoff", "quota-stop", "complete"),
                        required=True)
    parser.add_argument("--expected-id", required=True,
                        help="Current handoff-id, or 'none' for the initial save")
    parser.add_argument("--notes-file", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = save(args.root, args.agent, args.kind,
                      args.notes_file.read_text(encoding="utf-8"), args.expected_id)
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"Handoff save failed: {exc}\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
