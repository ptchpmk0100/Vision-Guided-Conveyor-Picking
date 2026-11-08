# V2 SORTING CELL — PROGRESS REPORT (for later check)

**As of 2026-11-07.** Autonomous run (Khoi's directive: proceed to C3, decide + log, don't
stop). This is the executive record: what was built, what is verified, and what remains
user-gated. Engineering detail → `V2_RESUME.md`; decision-by-decision rationale →
`V2_DECISIONS.md` (D1–D17); front-door usage → `README.md`. All work is in
`~/ros_test_ws_mirror`; the live `~/ros_test_ws` was never touched.

## The arc
1. **Phase A (original MVP → S22):** empty-world pick-and-place, repeat-without-respawn,
   camera-driven. 4/4 by ground truth. Evidence layer (SQLite + benchmark) added (A5).
2. **Belt-height redesign (Khoi's scene rework):** removed the ground cubes + ground bins;
   colour BOXES now sit on a belt-height deck and are sorted into colour-labelled TRAYS at
   belt height. Fully rebuilt and verified end-to-end (Stages 0/A/B/C/D/E).
3. **Phase B (systems depth):** hand-eye calibration (B1), behaviour-tree orchestration (B3),
   runtime validator (B4), reproducibility layer (B5). B2 (shape-aware grasp) skipped —
   uniform boxes removed the roll-off it was meant to fix.

## Verified results (ALL by Gazebo ground truth, judged independently of logs)
| Milestone | Result |
|---|---|
| S22 live baseline | 4/4 re-detect + sort, park at ready |
| Redesign Stage 0 — reach at belt height | GO, 42/42 probe legs |
| Stage A — scene geometry | 3/3 boxes @ z=0.320 × 2 |
| Stage B — perception (recalibrated) | 3/3 class-correct, 0 phantoms, XY ≤ 4.5 mm × 2 |
| B1 — hand-eye calibration | RMS 0.37 mm; recovers true extrinsic to 0.06° / 0.74 mm; repeatable; A/B ≥ hardcoded |
| Stage C/D — full colour sort | 3/3 correct tray × 2; pick 6/6 first-attempt; 0 errors |
| B3 — behaviour-tree sort | 3/3 correct tray × 2 with reactive recovery |
| B4 — runtime validator | flags all injected violations; OK on valid scene |
| Stage E — legacy ground pipeline | 3/3 (no regression) |
| Offline test suite (CI backbone) | ALL PASS (BT + validator + solver self-tests + py_compile) |

## Khoi's original redesign asks — all met
- Ground cubes + ground trays removed; the belt boxes ARE the pick targets. ✓
- Trays at the same height as the belt (on a tray table). ✓
- Trays colour-labelled to match the box they accept (coloured floor panels). ✓
- No redundant "move-to-object, fail, then sort" phase — uniform boxes removed the cylinder
  roll-off; each box is identified, picked, and sorted one at a time. ✓

## Key engineering lessons (so they are not re-learned)
- Judge by ground truth, never logs (every status line lied at least once).
- `pick`/`place` heights are relative to the object/target z → one code path for ground and
  belt height; belt height needed only a grasp-width retarget after Stage 0 proved reach.
- The detector uses true depth deprojection → raising objects moved only the z-gate; XY stayed
  accurate. The camera extrinsic is now *calibrated* (data-driven), not hardcoded.
- Flat calibration targets (not tall boxes) — parallax biased the centroid; flat tiles took the
  fit RMS from 3.0 mm to 0.37 mm. Fixing the data beats loosening a gate.
- Never add fill light / blue to the camera's view; never `pkill -f` a pattern matching your
  own shell (exit 144).

## Remaining — USER-GATED (outside the mirror-only / no-system-install autonomy boundary)
These were deliberately NOT attempted autonomously; each needs a decision or a resource:
- **C1 learned perception** (synthetic data → train seg/pose → ablate vs classical): needs
  training infra; low marginal value here (classical HSV is already 3/3, < 5 mm). Decide
  whether it is worth the effort before building.
- **C2 VLM task planning** (NL instruction → plan, ablate vs rule-based): needs an LLM API key
  + network.
- **Docker image build** (`docker build`), **CI run** (GitHub Actions), **demo video**, and
  **`git push` / remote + `v2.0` release tag publication**: all need your machine/account.
  A LOCAL `v2.0` tag has been created in the mirror; pushing it is yours to do.

## How to resume / run
`README.md` Quick Start. Headless scored sort: `tools/run_sort_line_check.sh 2`. Offline
tests: `tools/run_offline_tests.sh`. Recalibrate: `tools/run_b1_calibration.sh`. Resume
context: `V2_RESUME.md` then project memory.
