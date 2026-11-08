# Shared repository instructions

Read `AGENTS.md` for the workspace boundaries, build instructions, validation
requirements, and engineering constraints shared by Claude Code and Codex.
Follow the current user request; historical autonomy and usage-ladder notes in
older ledgers do not authorize additional work.

At session start, run `git status --short`, read `HANDOFF.md` when present, and
inspect the latest relevant `SESSION_LOG.md` entries. Verify the recorded state
and briefly acknowledge the next action before resuming.

Use the shared `handoff` skill at `.claude/skills/handoff/SKILL.md`, which links
to `.agents/skills/handoff/SKILL.md`. Follow its checkpoint, save, resume, and
90% five-hour/weekly quota policy. Keep the same root `HANDOFF.md` and
`SESSION_LOG.md` for both CLIs. The skill does not install a quota monitor;
unknown usage must stay unknown.

Checkpoint meaningful work and save the final state before ending a substantive
task. On a transfer or reported quota reaching 90%, save the handoff, warn the
user, end the active turn, and wait. Do not launch the other CLI automatically.
Preserve `.claude/settings.local.json` and unrelated work. Use repository files
for shared continuity; do not require private Claude memory to resume.
