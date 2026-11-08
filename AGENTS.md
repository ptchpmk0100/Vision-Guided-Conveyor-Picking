# Codex repository guidance

## Scope and collaboration

- `~/ros_test_ws` is the primary development repository;
  `~/ros_test_ws_mirror` is the testing and validation workspace. Work in the
  checkout selected by the current request. Promote tested mirror changes to
  the primary repository only when the user authorizes it. Preserve local
  settings and generated build/runtime files when synchronizing source.
- This repository is shared with Claude Code. Preserve its configuration in
  `.claude/`; do not rewrite it or depend on private `~/.claude` memory or skills.
  Read `CLAUDE.md` for the shared handoff entry point and project conventions.
- Start with `git status --short`. Preserve existing work from the user or another
  agent. Do not reset, clean, stash, or overwrite unrelated changes.
- Follow the current user task. Historical directives about unattended phases,
  Claude/Codex usage ladders, or manager roles are context, not authorization to
  resume old work or launch another agent. Codex can work directly and independently.
- For Codex setup requests, add only the necessary Codex guidance files; leave
  existing source, documentation, build files, and Claude settings unchanged.

## Read before changing behavior

At session start, after `git status --short`, read `HANDOFF.md` when present
and the latest relevant `SESSION_LOG.md` entries. Inspect the actual files and
evidence before resuming, and briefly acknowledge the next action. Follow the
shared handoff skill at `.agents/skills/handoff/SKILL.md`: checkpoint meaningful
work and save the final state before ending a substantive task. Both CLIs use
the same handoff and history files; do not keep per-agent copies.

At a reported five-hour or weekly usage of 90% or higher, stop starting normal
task work, save the final handoff, warn the user, and end the active turn. Only
handoff preparation follows the cutoff. Wait for the user to switch CLIs; do not
launch another CLI or agent automatically. This skill does not install a quota
monitor. Record unavailable usage as unknown and never claim a live cutoff is
enforced without a configured and tested monitor. Historical progress ledgers
remain references; the current request controls what work is authorized.

1. `README.md`: overview, entry points, and quick start.
2. `V2_PROGRESS.md`: recorded verification results and deferred work.
3. Relevant sections of `V2_RESUME.md` and `V2_DECISIONS.md`: engineering history,
   constraints, and rationale. Consult `V2_FULL_LOG.md`, `S21_DEBUGGING_LOG.md`, or
   `S22_REPEAT_LOOP_LOG.md` only when their detail is needed.

These records describe past runs. Inspect the current code and evidence before
claiming that behavior still passes. Resolve conflicting historical notes using
the relevant later decision and the actual implementation; the current user
instruction takes precedence. Do not resume deferred learned perception or VLM
work simply because an older ledger lists it as next.

## Project map

ROS 2 Jazzy / Gazebo Harmonic digital twin: a Franka Panda sorts red, green, and
yellow boxes from a belt-height staging deck into matching trays. Motion uses
MoveIt 2 (`moveit_py`); perception uses OpenCV HSV and depth deprojection.

| Path | Responsibility |
| --- | --- |
| `src/twin_action_demo/` | Launch files, worlds, object models, camera extrinsic, action nodes |
| `src/twin_description/` | Robot URDF and controller configuration |
| `src/twin_interfaces/` | Custom ROS action definitions |
| `src/twin_moveit_config/` | MoveIt planning and controller configuration |
| `src/twin_moveit_scripts/twin_moveit_scripts/` | Motion, perception, sort orchestration, collision scenes, validator, SQLite trials |
| `tools/` | Offline tests, calibration, headless harnesses, ground-truth checkers |
| `.github/workflows/ci.yml` | Offline test workflow |

The main cell uses `industrial_sorting_line_bringup.launch.py`,
`object_detector_sorting_line.py`, and `sort_tree_sorting_line.py` (behavior tree)
or `sort_task_sorting_line.py` (imperative). Shared grasp/place primitives live in
`move_to.py`. Preserve the legacy ground-cell pipeline when changing shared code.

## Build and run

Run from the repository root in a clean environment. Do not load `~/.bashrc`: it
may source an unintended workspace and silently select its packages. Source the
ROS base first, then only this checkout's overlay. Existing `build/`, `install/`,
and `log/` contents are generated artifacts, not source or proof of a fresh build.

```bash
source /opt/ros/jazzy/setup.bash
# Normal build when the remaining workspace dependencies are already available:
colcon build --symlink-install --packages-select twin_action_demo twin_moveit_scripts
# For a fresh checkout or changes involving local dependency packages, use:
# colcon build --symlink-install --packages-up-to twin_action_demo twin_moveit_scripts
source install/setup.bash
export GZ_VERSION=harmonic
```

Build other affected packages explicitly when changing interfaces, URDF, or
MoveIt configuration. Verify `ros2 pkg prefix <package>` points into this
checkout before running behavior checks. Avoid `set -u` around ROS setup scripts.

```bash
# Simulator and detector:
ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py headless:=true detector:=true
# Separate terminal, with the same base and overlay sourced:
ros2 run twin_moveit_scripts sort_tree_sorting_line
# Independent placement verdict while the simulator is still running:
python3 tools/check_sorting_line.py --placed
```

## Validation

- Offline code checks: `bash tools/run_offline_tests.sh`. This runs the behavior
  tree, validator, and calibration-solver self-tests plus Python compilation.
  It does not verify simulated motion or placement.
- For simulator behavior changes, use `bash tools/run_sort_line_check.sh 2` for
  consecutive fresh runs. The default sorter is imperative; set
  `SORTEXE=sort_tree_sorting_line` to exercise the behavior tree. Use a fresh
  `LOGDIR` (and `TWIN_DB` if needed) to preserve previous evidence.
- Judge success by Gazebo entity poses and the independent ground-truth checkers,
  not `pick=True`, `place=True`, or other node status messages. Inspect logs for
  errors, tracebacks, controller failures, and planning failures as a separate
  health check. Require at least two consistent runs for behavior changes.
- Read each harness before use. In particular, the sort harness prints checker
  results but does not reliably propagate all failures as a failing shell exit;
  inspect every verdict and log-health count rather than trusting its exit code.
- A historically observed MoveIt teardown exit code of 245 is only a known
  exception after completed work and a passing independent verdict. Do not
  suppress nonzero exits or new crashes on that basis.
- Documentation-only changes need a diff and reference review; do not launch a
  simulator or rebuild packages solely to validate text edits. Report checks
  actually performed and any limits; never present past results as new evidence.

## Engineering constraints and handoff

- Keep working components available while introducing replacements. Match nearby
  Python/ROS conventions and register new console entry points in the owning
  package's `setup.py` when applicable.
- Heights in shared pick/place primitives are relative to object/target z. The
  raised cell uses a deck top of 0.295 m and box center of 0.320 m; the legacy
  ground cubes use center z of 0.015 m. Do not mix those assumptions.
- Keep the calibrated camera transform in
  `src/twin_action_demo/config/overhead_camera_extrinsic.yaml`. Review and verify
  calibration candidates before promotion. Lighting, visible blue surfaces,
  object dimensions, and scene geometry can change detection or grasp behavior;
  revalidate affected perception, reach, and collision assumptions.
- Inspect processes before running simulations. Isolate concurrent runs with
  distinct `ROS_DOMAIN_ID` and `GZ_PARTITION` values. Stop only processes owned by
  the current run using verified PIDs; avoid broad `pkill -f` patterns.
- Keep changes scoped; exclude generated artifacts, credentials, and logs from
  commits unless the user explicitly requests particular evidence.
- When handing work between Codex and Claude, record changed paths, exact commands,
  observed results, evidence locations, and remaining work in the task response
  or an authorized shared handoff document. Do not rely on private agent memory.
  Update existing project ledgers only when the task authorizes such edits.

Codex instruction discovery reference:
https://learn.chatgpt.com/docs/agent-configuration/agents-md
