# V2 SORTING LINE — DECISION LOG

Autonomous-run decision ledger (Khoi, 2026-11-07): at every decision point, list the
options, reason + challenge them through the lens **"which choice has the most PRACTICAL,
INDUSTRIAL, REAL-WORLD impact (while staying feasible and verifiable)?"**, drive until ONE
option clearly stands out, record it here, and proceed — no pausing to ask (changes stay in
`~/ros_test_ws_mirror` only; `~/ros_test_ws` is never touched without explicit confirmation).

Format per decision: Options → Reasoning/Challenge → **Verdict** → consequence.

---

## D0 — Operating mode & usage ladder for the autonomous run (2026-11-07)
**Options:** (a) Claude writes all code while fresh, switch to Codex-main at 80% of the 5h
limit (literal reading); (b) Codex is the main executor/planner for the whole run, Claude
supervises/verifies/decides/runs sims.
**Reasoning/Challenge:** The binding constraint is "DO NOT STOP until C3", and Claude usage
is the scarce resource. I cannot measure "80% of the 5h limit" from inside the session, so a
threshold-triggered switch is not reliably actionable. A continuous A→C3 marathon burns
through any 5h window fast, landing in the supervise regime almost immediately regardless.
Option (b) maximises Claude longevity and keeps the high-value judgment (review, ground-truth
verification, decisions, reserved actions) with Claude throughout.
**Verdict: usage-state ladder (clarified by Khoi 2026-11-07).** The mode depends on Claude's
5h-limit state, so the session "almost always continues":
- Claude fresh (< 80%): **Claude completely** — Claude is the main executor/coder/planner AND
  verifier (do the work directly, no Codex round-trips).
- Claude at 80% AND Codex has budget: **Codex = main executor, Claude = verify/supervise.**
- Claude at 80% AND Codex not reset: **wait.**
- Claude fully out: **Codex completely.**  Both out: **wait.**  Claude resets: **Claude completely.**
(Supersedes the earlier "Codex drives the whole run" reading. Codex is still driven via the
`codex-manager` skill when the ladder calls for it; the mirror git baseline makes that safe.)

## D1 — Belt behavior (Khoi-selected 2026-11-07, logged for the record)
**Options:** static staging · index+stop conveyor · continuously moving belt.
**Reasoning/Challenge:** Moving-target picking is a different, far riskier problem class and
would wreck the verified grasp; a real sorting CELL commonly works off a staged infeed buffer
anyway. Static staging gives industrial realism (a pick buffer at a fixed station) without
the moving-grasp risk.
**Verdict: static staging** — fixed set of boxes at a belt-height pick zone, sorted until
clear.

## D2 — Box scheme (Khoi-selected 2026-11-07)
**Options:** identical colored boxes (color-only routing) · mixed shapes too.
**Reasoning/Challenge:** Mixed shapes reintroduce the cylinder roll-off/fling that needs the
deferred shape-aware grasp (B2); uniform cuboid cartons are also the most common real sorting
payload. Uniform boxes = reliable top-down grasp now, shape-sorting becomes a later S3 benchmark.
**Verdict: identical colored boxes**, routed by color (red/green/yellow → matching trays).

## D3 — Tray labeling (Khoi-selected 2026-11-07)
**Options:** colored floor panel · colored walls · colored floor + painted text.
**Reasoning/Challenge:** The tray must visibly declare the class it accepts (a real sortation
chute/bin is color/label coded). Colored floor keeps the saturated color below the detector's
object-height gate so it can never be mistaken for a box; walls/text add perception risk or
modeling cost for little extra realism now.
**Verdict: colored floor panel** per tray (gray walls), matching the accepted color.

## D4 — Infra: git-init the mirror (2026-11-07)
**Options:** pass `--skip-git-repo-check` to Codex · `git init` the mirror.
**Reasoning/Challenge:** The preflight flagged "no git → no diff review, no rollback", the
top operational risk for an unattended run. `git init` + a verified-state baseline commit
retires that risk and unblocks Codex, is local+reversible, and touches nothing in
`~/ros_test_ws`.
**Verdict: git init + baseline commit** of the verified A1–A5 state.

## D5 — Stage 0 reach gate: GO at belt height z=0.295 (2026-11-07)
**Question:** is belt-height grasp/release kinematically feasible, or must the plane be
lowered/moved? **Result (Gazebo ground truth):** 42/42 legs PASS (3 boxes + 3 trays + ground
control, approach/work/lift × 2 passes), gripper open confirmed, hand vertical (down_xy=0,
drop=165 mm), finger-mid within ~0 mm of the calibrated target; delta=−58.4 mm (sane).
**Verdict: GO — build the scene at z_surface=0.295 / box_center=0.320.** No fallback (lower
/ translate / pull-in) needed. Caveats logged: (i) free-space only — re-verify WITH the
table/belt/tray collision geometry at Stage C; (ii) validated 2 passes × 1 bringup (results
deterministic at ~0 mm); the consecutive-fresh-bringup rigor is applied at the functional
Stages C/D rather than spent here, to keep the marathon moving.

## D6 — Stage A geometry: PASS (2026-11-07)
New additive scene built directly (Claude-complete): `worlds/industrial_sorting_line.sdf`
(raised pick deck w/ collision, tray table, 3 collision trays with coloured floor labels),
3 box models (`obj_{red,green,yellow}_box.sdf`, 40×40×50 mm, dynamic), launch
`industrial_sorting_line_bringup.launch.py` (belt-height spawns, `detector:=` flag),
checker `tools/check_sorting_line.py`, runner `tools/run_sorting_line_check.sh`.
**Ground truth: 3/3 boxes settled at (0.560, ±0.110/0, z=0.320) on 2/2 consecutive runs,
0 legacy entities.** Box rest height proves the deck collision. Caught+fixed a checker
parser bug (setdefault/next-name vs the proven inline-capture parser; also read
dynamic_pose/info for moved objects) — scene itself was correct. Cosmetic caveat: static
trays/deck are omitted from the pose topics, so the "trays present" line reads NONE; tray
function is proven at Stage C by placement.

## D7 — Stage B detector: PASS (2026-11-07)
`object_detector_sorting_line.py` (belt-height recal of the multi detector: OBJ_HALF_HEIGHT
0.025, EXPECTED_CENTER_Z 0.320, ROI to the tray span, area 200-1600, blue removed, shape
forced "box") + `tools/check_detections_sorting_line.py`. **Ground truth, 2/2 consecutive:
3/3 detected, 3/3 class-correct (red/green/yellow_box), 0 phantoms, XY err 2.7-4.5 mm, Z err
0.0 mm.** The coloured tray-floor panels never phantom (computed z~0.272 vs gate 0.310, AND
~5850 px vs MAX_AREA 1600). Confirms the depth-deprojection reasoning: only the z-gate moved,
XY stayed accurate at the new height.

## D8 — Stage C/D full belt-height sort: PERFECT (2026-11-07)
`sort_task_sorting_line.py` (belt-height trays, 40 mm grasp gate via pick(expect_half_width=
0.020), tray-release clearance 0.035, collision-aware via `sorting_line_scene.py`), one minimal
backward-compatible change to `move_to.pick()` (grasp-width gate parameterised; default =
ground cube), `sorting_line_scene.py` (3 trays as hollow collision walls at z=0.315; deck+table
omitted like A3's ground plane), `--placed` mode in the checker, `tools/run_sort_line_check.sh`.
**Ground truth, 2/2 consecutive: 3/3 boxes in the correct tray (red/green/yellow), all z=0.320;
0 system errors / 0 planning failures / 0 tracebacks / 0 fatals.** Khoi's 4 asks all met:
belt boxes are the targets, trays at belt height, colour-labelled trays, no redundant failed
phase (uniform boxes remove the cylinder roll-off). pick() heights are all relative, so belt
height "just worked" once the grasp gate was retargeted — Stage 0 had already proven reach.

## D9 — Stage E regression + REDESIGN-PHASE-A REVIEW (2026-11-07)
**Stage E (backward-compat gate):** old ground A4 sort still **3/3 in correct bin** by ground
truth after the pick() change (cubes clean; cylinder fails attempt 1 then recovers — the
documented B2 behaviour, unchanged). The pick() parameterisation is backward-compatible.

**Review (Khoi rule 8, after the redesign's Phase-A-equivalent = Stages 0/A/B/C/D/E):**
- Code: additive, old pipeline intact. Documented assumption: `sorting_line_scene` models tray
  WALLS, omits pick deck + tray table from MoveIt collision (A3's ground-plane reasoning:
  fingers graze those surfaces). Empirically safe (6/6 cycles, 0 planning failures); re-check
  if box/tray poses move.
- Errors (all resolved, none open): probe panda_link8 absent -> finger-midpoint (precheck
  caught it); finger-offset miscalibration -> ground-calibrated delta (anchor caught it);
  checker setdefault parser bug -> inline-capture + dynamic_pose/info.
- Metrics: pick 6/6 all + 6/6 FIRST attempt (uniform boxes removed the roll-off and the
  "redundant failed phase"); 3/3 correct tray x2; ~58 s/box, ~206 s/run; 0 logged-vs-truth
  divergence.

**PLAN REVISION (industrial-impact lens): re-order Phase B to B1-first.**
- **B2 (shape-aware grasp) DEPRIORITISED** — it was top priority only because of the cylinder;
  the redesign's uniform boxes make grasping deterministic (6/6 first attempt), so B2 is no
  longer blocking. It returns only if shape-variety sorting (S3) is reinstated.
- **B1 (hand-eye calibration) = new top priority.** The system still uses a HARDCODED
  optical->world transform (`world_x=0.4-y_opt`, ...). That is fragile and not real industrial
  practice; a calibration routine that computes the camera extrinsic (and drops the hardcode)
  is the single highest real-world-practice gain and de-risks every downstream change. Risk:
  it touches working perception (<5 mm today) -- will be gated behind the same 3-legged
  ground-truth validation and kept additive (new node; old mapping stays until the calibrated
  one beats it on ground truth).
- Then B3 behaviour tree (py_trees orchestration), B4 validator node, B5 Docker+CI+README.

## D10 — B1 hand-eye calibration: plan APPROVED (2026-11-07)
Codex planned it (read-only). **Verdict: approved** — method is sound and industrially honest:
Kabsch rigid fit (NO scale; scale kept only as a diagnostic that fails on depth/intrinsic
error), optical TOP-face paired with world TOP-face (avoids a hidden 25 mm Z bias — the single
biggest correctness trap, correctly caught), 9-box grid + 3 raised pedestals for a non-degenerate
3D fit (singular-value-ratio gate), Hungarian one-to-one colour association (nominal transform
used only to associate, never as fit input), additive detector `extrinsic_file` param with
hardcoded FALLBACK on absence but FAIL-LOUD on a requested-but-malformed file, and an A/B gate
requiring the calibrated transform to be no worse than the hardcoded baseline + recover the known
nominal extrinsic within 2°/20 mm (treats a wild "correction" as a bug, not a win).
**Manager streamline (pace):** build Stages 1+2 together (collector + Kabsch solver, verify the
fit recovers the nominal extrinsic = the key proof), then Stages 3+4 (detector integration +
production A/B), then promote the YAML. Codex builds; Claude runs the sim + owns the ground-truth
verdict (ladder: Codex-main while the marathon conserves Claude).

## D11 — B1 Stage 1+2 first run FAILED (correctly) → fix = flat tiles, not looser gates (2026-11-07)
Both captures collected 12/12 cleanly, but the Kabsch fit failed its gates: RMS 3.0 mm (>2),
max 4.2 mm (>4), **diagnostic scale 1.0226 (>[0.998,1.002])**. Root cause, confirmed by
per-target residuals growing monotonically with radial distance from the camera axis (0.2 mm
at 30 mm → 4.2 mm at 219 mm): the 50 mm-tall boxes show their SIDE faces when off-centre, so
the blob centroid is biased outward by a viewing-angle amount — a radial error that reads as a
2.3% scale. The scale-free solver correctly refused to absorb it into R,t.
**Options:** (a) loosen fit gates to ~4 mm and lean on the A/B test; (b) fix the
correspondences with FLAT calibration targets. **Verdict: (b)** — real hand-eye calibration
uses FLAT targets precisely to avoid parallax; loosening the gate would bake the bias into the
extrinsic and is the "relax a safety net to pass" anti-pattern. Fix: swap the 12 tall boxes for
12 flat 40×40×4 mm colour TILES at the same grid/raised poses; parameterise the collector's
target half-height (0.025 box → 0.002 tile). Keep all gates strict. Re-run must pass AND
recover the known nominal extrinsic.

## D12 — B1 Stage 1+2 with flat tiles: PASS (2026-11-07)
Flat 40×40×4 mm tiles removed the parallax. Gated Kabsch fit over 2 captures (24
correspondences): **RMS 0.370 mm, max 0.530 mm, scale 0.9980, geometry 0.19, det=1,
orthonormality 2e-15.** Repeatability dR=0.0°, dt=0.0 mm, cross-RMS 0.370 mm. **Recovered the
known-true extrinsic to 0.060° / 0.74 mm** (R≈[[0,-1,0],[-1,0,0],[0,0,-1]], t≈[0.400,0.000,
1.200]). Genuine data-driven calibration, not circular. Candidate YAML written. Next: Stage 3
(detector loads the extrinsic from config, hardcoded fallback) + Stage 4 (A/B vs baseline +
full sort with the calibrated detector).

## D13 — B1 Stage 3+4 A/B: PASS → B1 COMPLETE (2026-11-07)
Detector gains optional `extrinsic_file` param (empty -> hardcoded fallback; valid -> load+
apply `p_world=R@p_opt+t`; malformed/missing -> FAIL loud, verified). Candidate promoted to
`config/overhead_camera_extrinsic.yaml` (installed via setup.py). **A/B by ground truth, x2:
calibrated detector 3/3 class-correct, 0 phantoms, worst XY 4.4 mm (vs hardcoded 4.5 mm — no
worse); detector log confirms it loaded the YAML (not fallback); calibrated full sort 3/3
correct tray, 0 errors.** The hardcoded optical->world transform is replaced by a genuine
data-driven calibration (recovered to 0.06deg/0.74mm), with the formula kept as fallback.
**B1 done.** Next per the re-ordered plan: B3 (py_trees behaviour tree) — B2 skipped (uniform
boxes). Fewer phantoms/latency, honest industrial calibration path = the real-world gain.

## D14 — B3 behaviour tree: self-contained, NO py_trees system install (2026-11-07)
`py_trees`/`py_trees_ros` are not installed. **Options:** (a) `apt/pip install` py_trees —
a system change OUTSIDE the mirror, which the autonomy grant ("changes only in
ros_test_ws_mirror; do not jeopardise the system") does not cover; (b) a lightweight BT in
pure Python inside the mirror (Status SUCCESS/FAILURE/RUNNING, Sequence/Selector/Retry, leaf
behaviours wrapping the sort primitives), no dependency. **Verdict: (b).** It delivers B3's
real industrial value — modular, reactive, inspectable orchestration WITH recovery branches —
with zero system change, honouring the boundary. py_trees can be swapped in later if Khoi
approves a system install; the BT interface is kept small so that swap is cheap. Verified by
the same ground-truth sort check (3/3 correct tray). (B2 shape-grasp stays skipped.)

## D15 — B3 behaviour tree: PASS (2026-11-07)
`behaviour_tree.py` (dep-free Status/Sequence/Selector/Retry/Leaf + self-test) +
`sort_tree_sorting_line.py` (entry `sort_tree_sorting_line`). BT self-test passes;
tree-driven sort = **3/3 correct tray x2, 0 errors**, tick log shows
`Sequence[GetFreshDetections -> Retry(Pick,2) -> Place -> ReturnReady]` executing with
recovery. Same verified primitives, reactive+inspectable orchestration added. B3 done.
Next B4 (validator node), then B5 (file artifacts). B2 skipped.

## D16 — B4 validator node: PASS (2026-11-07)
`sorting_validator.py` (entry `sorting_validator`): independent runtime monitor on
`/detected_objects/array` -> `/validation/report` (JSON). Invariants: FR-2 workspace envelope
+ belt-z, FR-3 known-class + overlap, FR-8 wrong-tray. **Self-test flags all 5 cases
(valid->OK, OUT_OF_ENVELOPE, WRONG_TRAY, UNKNOWN_CLASS, OVERLAP); live on the valid scene it
reports detections=3 violations=0 = OK.** Observe-only (never drives the robot / mutates the
world). B4 done.
**Process note:** I hit the `pkill -f sorting_validator` landmine (exit 144 — pattern matched
the calling shell), as V2_RESUME.md §2 warns. Re-ran killing only by PID/PGID. Lesson re-learned:
never `pkill -f` a pattern that can match the running command.

## D17 — B5 reproducibility layer: DONE (files) + PHASE-B REVIEW (2026-11-07)
Added `README.md` (project narrative + verified results + run commands), `Dockerfile`
(ROS2 Jazzy + Gazebo Harmonic + colcon build), `.github/workflows/ci.yml` (runs the offline
suite), and `tools/run_offline_tests.sh` — **the offline suite runs ALL PASS now** (BT,
validator, solver self-tests + py_compile of every module). Image build / CI run need
Docker / a GitHub runner (external — flagged, not exercised here).

**PHASE-B REVIEW (Khoi rule 8, after B5):**
- Code: all Phase-B work is additive and ground-truth-verified — B1 (calibration, fail-loud +
  fallback), B3 (dep-free BT), B4 (observe-only validator), B5 (docs/CI/offline suite). The
  verified A/redesign pipeline is untouched; offline suite guards regressions cheaply.
- Errors (all resolved): B1 tall-box parallax (fixed with flat tiles, gates kept strict);
  `pkill -f` landmine (recovered, PID/PGID kills only now). None open.
- Plan revision for Phase C (industrial lens + the mirror-only/no-system-install autonomy
  boundary): **the autonomously-completable Phase-C deliverable is C3's written report + a
  local v2.0 git tag.** C1 (learned perception) needs training infra and its ablation is low
  marginal value (classical HSV already 3/3, <5 mm) — DEFER as user-gated. C2 (VLM task
  planning) needs an LLM API key + network — user-gated. Demo video + `git push` — user-gated.
  So: finish with the report + tag autonomously; hand the external-dependency bricks to Khoi
  with a clear menu.

## D18 — mode switch: COMPLETELY CODEX (2026-11-07)
Khoi: "switch to completely codex, commit everything and log everything." From here Codex is
the sole EXECUTOR (all building/writing); Claude only frames, verifies, and performs reserved
actions. Local git commits remain with the manager (a commit is not a reserved push). First
task under the new mode: Codex writes the exhaustive session log `V2_FULL_LOG.md` by
synthesising the authoritative records (this file, V2_RESUME.md, V2_PROGRESS.md, README.md,
git history, logs/v2/*.out); the manager verifies accuracy and commits.

---
(Appended as the run proceeds.)
