---
name: handoff
description: Save or resume shared working context when switching between Claude Code and Codex, checkpointing substantive repository work, ending a session, or responding to a reported five-hour or weekly quota reaching 90 percent.
---

# Shared Claude Code / Codex handoff

Use one `HANDOFF.md` and one append-only `SESSION_LOG.md` at the repository root.
Both CLIs load this same skill folder; never create per-agent copies of the
documents. Follow the current user's scope and the repository's instructions.

## Start or resume

1. Run `git status --short` first. Read `HANDOFF.md` and the relevant latest
   `SESSION_LOG.md` entries, then inspect the recorded files and evidence.
   If no handoff exists, use the current request and repository guidance.
2. Acknowledge the previous agent's completed work, remaining work, and your
   next action briefly. Resolve stale notes against the actual checkout and
   current instructions. A log entry is context, not new authorization.
3. Carry forward unfinished items and constraints when saving the next entry.
   Never restart historical deferred tasks solely because a ledger lists them.
   If the previous task is complete, wait for or follow the new user request.

## When to save

- Checkpoint after a meaningful milestone or costly verification and before
  the final response for a substantive task. Include the final verification.
- On an explicit handoff, session end, or switch request, save a full handoff
  and end the active turn. Do not start the other CLI yourself.
- If either the active CLI's five-hour or weekly usage is reported at or above
  **90%**, immediately stop starting normal task work. Only inspect state and
  finish saving the handoff, warn the user, and end the active turn. Do not
  continue toward 95% or use the remaining quota for another task.
- Quota means consumed allowance, not elapsed time or context-window fullness.
  Record the percentage, source, observation time, and reset time when available.
  Missing telemetry is **unknown**, never zero; never invent a quota reading.
  A previous session's reading must not be treated as a live measurement.

This skill does not poll account usage, install hooks, interrupt an in-flight
model request, or stop the CLI process. It implements the procedure when
invoked or when reported usage is available to the agent. A guaranteed external
trigger requires a separately installed and tested usage monitor. Do not claim
that background monitoring is active merely because this skill is installed.

## Write and save

Prepare a UTF-8 Markdown notes file outside the repository, using the following
sections. Write concrete facts; mark unavailable information as unknown and
unperformed checks as not run. Include enough context to resume without the
conversation, and link detailed output rather than copying entire transcripts.

- `## Task and constraints`: current user objective, scope, active/completed/
  paused state, and relevant authorization boundaries.
- `## Changes and decisions`: changed paths, reasons, completed steps, failed
  approaches worth preserving, and pre-existing work that must be kept.
- `## Verification and evidence`: exact commands, environment, observed results,
  failures, and durable evidence paths. Separate new results from historical ones.
- `## Running operations`: owned process/session IDs, command, working directory,
  logs, ROS domain/Gazebo partition, and whether the operation is still running.
  Do not terminate unrelated processes or assume a recorded PID is still owned.
- `## Usage`: observed five-hour/weekly percentages, source, timestamp, reset
  times, or unknown. State whether external monitoring is actually configured.
- `## Remaining work and blockers`: unfinished changes, uncertainties, required
  resources, and anything the next agent must avoid repeating.
- `## Next action`: the next concrete command or action, or state that the task
  is complete and a new user request is needed. Include the proposed next CLI
  for a transfer, without claiming its quota is available unless checked.

Exclude credentials, private agent memory, secret environment values, and full
transcripts. Capture only task-relevant process details. Preserve unrelated
changes and existing project ledgers. Do not commit or push as part of a handoff.

Read the `handoff-id` comment at the top of the existing `HANDOFF.md`. From the
repository root, run the helper bundled with this skill (resolve its path from
the loaded skill location if this skill is installed elsewhere):

```bash
python3 .agents/skills/handoff/scripts/save_handoff.py \
  --root . --agent codex --kind handoff \
  --expected-id CURRENT_HANDOFF_ID --notes-file /tmp/handoff-notes.md
```

Use `--agent claude` for Claude Code. Supported kinds are `checkpoint`,
`handoff`, `quota-stop`, and `complete`. Use `--expected-id none` only when
there is no `HANDOFF.md`. The helper adds a UTC timestamp, entry ID, branch,
commit, and current Git status; appends the entry to `SESSION_LOG.md`; and
atomically replaces `HANDOFF.md`. It never stages files, runs tests, or controls
processes. If another writer updated the handoff, reread and reconcile before
retrying; never blindly overwrite the newer state.

Read back both documents and check that their latest entry IDs match the
helper's output. The history is written first: after an interrupted save,
inspect the latest history entry to recover a newer entry missing from the
snapshot. Never announce that saving succeeded after a helper error.

After `quota-stop`, report the observed quota and reset time (if known), the
saved paths, and ask the user to open the next CLI in this checkout and read
the handoff. Leave any running operation clearly documented. No normal task
work follows the saved transfer. At the next start, recheck live usage before
resuming after a quota stop; a new user request alone does not reset a quota.
