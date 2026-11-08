# S21 Debugging Log: perception-driven pick-and-place in ROS 2 + Gazebo

Written 2026-11-06 by Claude Code (Sonnet 5.5), at Khoi's request, after the session.
Location: `~/ros_test_ws_mirror/` (the live workspace `~/ros_test_ws` is read-only by Khoi's rule and was not modified).

> **STATUS UPDATE, READ THIS FIRST.** Sections 0-8 were written before the work was finished and
> are kept as the historical record. **Section 9 supersedes them** wherever they disagree. In particular
> section 0 ("not solved"), section 6 and its next steps, and the key numbers in section 8 are out of date.
> Final result: 12 of 12 consecutive cycles passed, confirmed by simulator ground truth (section 9.6).

This is a general-purpose record. Sections 1-5 are the story in order, including every wrong turn.
Section 6 is the unresolved list as of that point. Section 7 is the generalised method. Section 8 is
reproduction material. Section 9 is the completion of the work and the current state.

Evidence labels used throughout:
- **[VERIFIED]** observed directly in a command's output this session.
- **[INFERRED]** follows from verified facts but was not tested on its own.
- **[UNTESTED]** a hypothesis only.

---

## 0. One-paragraph summary

A pick-and-place cycle (`pick_and_place box table`) worked once, then failed on the second run
with `pick phase: FAILED [no_object_grasped]` after the arm descended to z=0.523 for a cube that
really sat at z=0.015. The handoff offered two hypotheses (the cube sits on a real elevated table, or the camera depth
math is wrong). Both were wrong. The real cause was a **colour-mask bug in the perception node**:
its "blue" test also accepted neutral grey (120,120,120), the colour of the Panda's dark link
bands. When the hand covered the cube, the mask filled with arm pixels and the node published the *arm's* height as the
cube's pose. A second, separate problem remains: grasps from the `table` coordinate still fail, and the
evidence points at one finger joint that moves far slower than the other. That second problem is **not solved**.

---

## 1. Starting situation

- User ran a long chat on claude.ai, which cannot touch the machine. It produced `CONTINUATION_S21.md`
  (found at `~/Downloads/CONTINUATION_S21.md`) containing: a status report, live-debug narrative,
  a "next brick" and a start command. Its own header said the log excerpts are primary evidence and
  its analysis is inference to be re-checked. That framing is correct and was followed.
- Stack: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2 (`moveit_py`), `ros2_control`,
  a 7-DOF Panda with two independently commanded finger joints, an overhead RGB-D camera,
  and a 3 cm blue cube.
- Constraints in force: the standing rule that `~/ros_test_ws` is never modified (no writes, no builds,
  no deletes; clone to experiment); the Claude/Codex/Perplexity routing policy; project accuracy rules
  (a claim of "done" needs a command and its output; two consecutive logged passing runs
  are required for DONE).
- Routing decision: Claude did everything. Reason: the cause was unknown, so by the policy the diagnosis stays with Claude.
  Codex was not used and no preflight was run (nothing delegable was specified).

### What the handoff claimed vs what was true

| Handoff claim | Finding |
|---|---|
| Detector terminal was never captured, so its sanity log line was never seen | **[VERIFIED]** `logs/S21_detector_run.log` exists (2 lines, written 16:48) and already shows `depth=0.490 -> world=(0.328, 0.185, 0.695)` for a "first detection". The evidence was there; it just was not read |
| Hypothesis (a): the box rests ~0.43 m up on a real table model | **[VERIFIED false]** the world file (`worlds/sensors_empty.sdf`) contains only a ground plane. "table" is only the tuple `(0.35, 0.2, 0.015)` in `move_to.py` (`_KNOWN_TARGETS`) |
| Hypothesis (b): depth-to-world math has a bug | **[VERIFIED false]** on a clean scene the detector read (0.442, 0.002, 0.015) against a true pose of (0.45, 0.0, 0.015) |
| `object_detector` is missing from the launch file | **[VERIFIED true]** (grep result quoted in the handoff, consistent with the launch file contents) |
| Run 1 "succeeded" (unlogged) | **[UNVERIFIED]** by the handoff itself. Later reproduced under a patched system (see 4.14), but the original unlogged run is still unverified |

---

## 2. The approach (brainstorm before action)

The user asked for brainstorming first, then execution. The approaches considered, in the order
they were weighed:

1. **Read what already exists before running anything.** Cheapest, zero risk. The logs directory and the
   detector source were read first. This paid off immediately (the existing detector log contradicted the handoff).
2. **Get ground truth from the simulator, not from the system under test.** Gazebo publishes true model
   poses on `/world/<world>/dynamic_pose/info`. Comparing the detector's output with that is the
   decisive test of "is perception wrong?". Without it, "the box is really up there" and "the detector is wrong"
   cannot be told apart.
3. **Test the suspected component in isolation on a clean scene** (detector alone, arm out of the way,
   cube at its spawn pose) before testing the whole cycle.
4. **Look at the raw sensor data** (save an RGB + depth frame and apply the exact mask code to it) once a
   number looked wrong that no hypothesis explained.
5. **Reproduce the user's two-consecutive-run scenario** with every terminal captured to a file.
6. **Fix in a copy, replay offline first, then validate live.** Offline replay of saved frames is deterministic and
   cannot be confused by sim timing.
7. Rejected: spawning a Codex lane. The cause was unknown, and a Codex session costs ~30k-80k tokens
   before useful work, with no benefit for interactive sim debugging.
8. Rejected: asking the user to run commands. The tools were available on the same machine.

---

## 3. Ordered narrative, with every dead end

### 3.1 Finding the file (first small failure)

- `find / -name CONTINUATION_S21.md` ran past the 120 s tool timeout and was auto-backgrounded.
  **Lesson:** never search `/`. Stopped the task, then searched `~` with `-maxdepth 4`; found it in `~/Downloads`.

### 3.2 Reading the state (read-only)

- Read the memory rule (read-only workspace, clone to experiment), listed `logs/`, read `object_detector.py`
  and grepped `move_to.py` for `table`.
- Key facts established from source alone:
  - The camera is a URDF link at world (0.4, 0, 1.2) looking straight down, clip 0.1-3.0 m, 640x480, 60 degree FOV.
  - The detector's mapping is `world_x = 0.4 - y_opt; world_y = -x_opt; world_z = 1.2 - z_opt`.
    I re-derived it from the URDF rotation (pitch +90 degrees: camera forward becomes world -z, image-up becomes
    world +x, image-right becomes world -y) and it is **[VERIFIED]** consistent.
  - The detector deliberately subtracts a half cube height to convert top-face z to the resting centre.
  - The mask test was `B>=120 & R<=120 & G<=200`. **At this point I read it as "deliberately loose" (the file's own words)
    and did not yet see the problem.** It only became visible with data (3.9).

### 3.3 Bringing the system up (all from the read-only install)

- Launched `panda_bringup.launch.py` with `nohup`, output to the scratchpad, sourced from
  `~/ros_test_ws/install`. Running installed packages does not write to the workspace (ROS logs go to `~/.ros/log`).
- Verified: three controllers active, `/overhead_camera/*` topics present.
- The cube is **not** spawned by bringup. It is spawned by hand:
  `ros2 run ros_gz_sim create -world empty -name b -file ~/ros_test_ws/grasp_box.sdf -x 0.45 -y 0 -z 0.03`
  (command found in the S19 handoff). It settles to z=0.015.

### 3.4 Test 1: is the detector's math right? (hypothesis b)

- Ran the detector in a captured terminal; read ground truth with
  `gz topic -e -n 1 -t /world/empty/dynamic_pose/info` filtered to the model named `b`.
- Result: detector `(0.442, 0.002, 0.015)` vs truth `(0.45, ~0, 0.01499)`.
- **Conclusion:** z and y are right; x is 8 mm low. Hypothesis (b) as stated (large z error from bad math) is false.
  The 8 mm x bias was noted and left unexplained at this point (explained in 3.10).

### 3.5 Test 2: run the cycle unpatched (a new failure appears)

- `pick_and_place box table`, output captured. It **failed**, but differently from the handoff:
  perception was correct (0.442, 0.002, 0.015) and the grasp stage failed with
  `gripper reached width=0.0283 without stalling`. The cube had been pushed 11 mm in y.
- Immediately after, the detector reported `(0.271, -0.049, 0.570)` while the cube truly sat at (0.45, -0.011, 0.015).
  A detector that said z=0.015 minutes earlier now said z=0.57 for a cube that never left the ground.
  **This was the first direct reproduction of the handoff's "z is wrong" symptom**, and it showed the error
  depends on *arm state*, not on where the cube is.

### 3.6 Wrong turn 1: "the cube is yawed 45 degrees"

- Reasoning: during an earlier grasp the fingers stopped at 0.0333, while the S19 handoff recorded real holds at
  0.012-0.015. A 3 cm cube rotated 45 degrees is 0.0424 m across its diagonal, a plausible blocker.
- Test: read the cube's quaternion from the sim. z=-0.00038, w=1, which is about -0.04 degrees of yaw.
- **Refuted.** Cost: one command. The habit of asking "what single reading would refute this?" made it cheap.

### 3.7 Wrong turn 2: "it's the sim running slowly (wall clock vs sim clock)"

- Reasoning: `move_to` uses wall-clock waits and a "settle" loop; a slow sim could freeze fingers mid-stroke while the loop
  declares them settled.
- Test: three RTF samples from `gz topic -e -t /stats`: 0.99, 1.23, 0.60. Inconclusive; I noted it and moved on.
  (Revisited properly in 3.12-3.14, where it turned out to be real but *not the cause of the failure*.)

### 3.8 Direct gripper test, and a self-inflicted shell kill

- With the arm still at the grasp pose, sent open then close to the gripper action directly.
  Both returned `Goal successfully reached` and the cube had moved again.
- The same command ended with exit code 144. Cause: `pkill -f "topic echo /joint_states"` matched my **own bash command line**
  (the pattern text was inside it). The recording file was also empty because the `--field position` output format
  was not what my parser expected.
- Afterwards: read the finger TF frames. The finger frames were 0.030 m apart, exactly the cube width, so the
  fingers *had* closed on the cube in that manual test. This showed a grasp was physically possible at that pose
  and that the pipeline's verdict could be wrong. It did not yet say why.

### 3.9 Look at the actual pixels (the turning point)

- Wrote `grab.py` (RGB + depth from the live topics) and applied the detector's exact mask to it.
  First attempt failed: `ModuleNotFoundError: rclpy`. I had run python with `-I` (isolated mode, a safety habit for
  untrusted files), which also ignores `PYTHONPATH`, the way ROS exposes `rclpy`. Reran without `-I`
  (my own script, so safe).
- Output on the failing state:
  - 689 mask pixels, bounding box u 269-456, v 194-479 (huge for a 3 cm cube that should be about 14 px wide).
  - Centroid (364, 355), **and the centroid pixel was not in the mask** (`mask[v,u] == False`).
  - The sampled mask pixels were `[120, 120, 120]`: neutral grey.
  - Depth at the centroid 0.616 m, depth over the mask min 0.564 / median 0.585 / max 1.2.
- Viewed the saved PNG: the Panda's hand and wrist cover the cube's location, and the cube is not visible at all.
- **Root cause found [VERIFIED]:** the mask accepts R=G=B=120 (satisfies `B>=120`, `R<=120`, `G<=200`). With the cube hidden, arm
  pixels dominate the mask; the centroid of a scattered blob lands wherever; depth there belongs to the arm. The node
  then publishes a confident wrong pose, which breaks its own documented rule "no detection, publish nothing".

### 3.10 Quantifying the separation between cube and arm

- Reset the scene (open gripper, `named ready`, set cube pose via the `/world/empty/set_pose` service) and grabbed a clean frame.
- Split the loose mask by depth (cube at ~1.17 m, arm below 1.1 m):
  - Cube: 196 px, all exactly RGB (104, 172, 216): B-R = 112, B-G = 44.
  - Arm: 1209 px passing the old test, R 45-120, G 120-173, B 120-203, B-R from 0 to 155, B-G from 0 to 31.
    The B-R=155 pixels are the hand's blue indicator dots, which is why B-R alone is not enough.
- Design consequence: require blue to beat red by a lot **and** green by a clear margin. B-G is the discriminating
  channel (cube 44 vs arm max 31). The margin is only 13, so I chose thresholds B-R >= 80 and B-G >= 36
  and recorded the arm's worst case in the comment next to them.
- Side effect explained: the earlier 8 mm x error came from arm pixels leaking into the loose mask and dragging
  the centroid. **[INFERRED]** (the offline replay in 3.11 shows the cube-only mask gives x=0.451, which matches).

### 3.11 The fix, built in a copy and replayed offline first

- Cloned with `rsync -a --exclude build --exclude install --exclude log` to `~/ros_test_ws_mirror`.
- `object_detector.py` changes:
  1. Colour test replaced by blue-dominance (`B>=100`, `B-R>=80`, `B-G>=36`).
  2. Depth taken as the **median over the mask's own pixels** instead of `depth[v,u]` at the centroid
     (a centroid need not lie on the object when the blob is non-convex or partly occluded).
  3. A plausibility gate: if the perceived centre z is not within 0.04 m of 0.015, log a throttled warning
     and publish nothing. This restores the "unknown beats wrong" contract.
- Offline replay of the two saved frames through the patched code:
  - clean frame: `(0.451, -0.000, 0.015)` (truth 0.45, 0, 0.015)
  - the occluded frame that previously gave z~0.57: **no detection**.
- `move_to.py` change in `pick()`: move the "home to ready" step **before** `resolve_object`. Reason: the previous cycle leaves
  the arm parked near the cube, and the old code read the camera before moving it away. Also changed
  `object_xyz=xyz` to `None` in the ready-pose failure result, because `xyz` no longer exists at that point.
- Both files passed `py_compile`.

### 3.12 My own process mistakes (documented because they cost time)

1. `kill $(pgrep -f "[o]bject_detector")` inside a command that also contained `object_detector.py`
   killed the calling shell (exit 144). The bracket trick avoids matching the `pgrep` process but not the
   outer shell's own command line, which contains the full text.
2. `kill $(pgrep -f "gz sim gui" | head -1)` was meant to stop only the Gazebo GUI. It matched a parent
   wrapper first, and the **entire simulation died**. The follow-up RTF script then blocked on `readline()` with no data and
   the tool backgrounded it. It was my own sim, so no user data was lost, but the run had to be relaunched.
3. A heredoc terminator typed incorrectly (`EOF` inside the heredoc), producing a Python NameError. Cosmetic.
- Rule adopted: for process control use PIDs read from `ps -eo pid,args | awk '/pattern/ && !/awk/'`, or
  keep the kill command in a separate tool call from any command whose text contains the pattern.

### 3.13 Live validation, cycle 1 and cycle 2

- Ran the patched detector from the mirror (`python3 <mirror>/object_detector.py`) and the patched `move_to.py` from the mirror
  (`python3 <mirror>/move_to.py pick_and_place box table`). No build was needed, and running a script directly from the
  mirror leaves the installed workspace untouched.
- Cycle 1 (cube at spawn point (0.45, 0)): `resolve_object: live 'box' @ (0.451, -0.000, 0.015)`,
  grasp confirmed at width 0.0139, `pick=True, place=True`. **[VERIFIED]**
- Cycle 2 (cube now at the `table` point, arm just returned): `resolve_object: live 'box' @ (0.351, 0.205, 0.015)` against
  truth (0.3496, 0.2037, 0.015): about 1.5 mm. **The original z=0.43 failure no longer occurs.**
  The pick **still failed**: `gripper reached width=0.0236 without stalling`. **A second defect exists.** [VERIFIED]

### 3.14 Investigating defect 2

Facts gathered (all [VERIFIED] unless marked):

1. Cube yaw after placement: z-quaternion 0.00099, about 0.11 degrees. Not a rotation problem.
2. A scripted close at the failing pose (`closetrace.py`, which logs finger joints against time):
   finger 1 plateaued at 0.0239 while finger 2 continued to 0.0063. The pair sums to 0.0302, which is the cube width,
   so the cube *was* gripped, off-centre.
3. In that trace, 6.3 s of wall time covered only about 2.3 s of sim time. RTF about 0.4.
4. `gz` `/stats` over 12 s: mean RTF 0.31, min 0.10, max 0.66, with load average 11 on 12 cores
   (the Gazebo server near 200% CPU, the GUI 112%, a browser also busy).
5. Relaunch without the GUI (a scratch copy of the launch file with `gz_args` set to `-r -s --headless-rendering`):
   mean RTF 0.99. Controllers came up and the camera still rendered headless (the detector found the cube).
6. Headless, patched, two consecutive cycles: **cycle 1 pass** (width 0.0129, `pick=True, place=True`),
   **cycle 2 fail** with the identical `width=0.0236` signature. So **slow-sim timing is not the cause** of the grasp failure
   (it is a real environmental problem, but a separate one).
7. `move_to.py` line 50: `FINGER_JOINT = "panda_finger_joint1"  # the one we read width from`.
   Grasp detection reads **only finger 1** even though both fingers are independently commanded.
8. Free-air close at the table pose (cube moved away to (0.9, 0.6) with `set_pose`): finger 2 reached 0.0 in about 2.8 s
   of sim time; finger 1 went from 0.0344 to only 0.0188 in about 3.3 s. So finger 1 is slow **without any cube**.
9. Free-air close at the `ready` pose: finger 1 sat at 0.0 and did not follow an open command, while finger 2 tracked
   normally. So the slowness does not depend on arm pose.
10. URDF: both finger joints have identical limits (effort 20, velocity 0.2) and `damping="0.3"`; axes are opposite in sign
    (`0 1 0` and `0 -1 0`). So the asymmetry is not a trivial URDF difference.

Interpretation [INFERRED, not tested]: with finger 1 lagging, a cube not exactly centred is pushed by the fast finger before the
slow one arrives; width is read from the slow joint, so the stroke looks like it "stopped short, then the hold stroke reached its
target exactly", which `move_gripper` classifies as a spurious stall and `pick()` reports as "nothing grasped".
That fits every observation but the direct test (compare commanded vs measured trajectory per finger, and a
two-finger width read) was not run.

### 3.15 Closing out

- Wrote the patch as a unified diff: `~/ros_test_ws_mirror/s21_detector_fixes.diff` (149 lines).
- Copied all evidence to `~/ros_test_ws_mirror/logs/s21_claude_code/`.
- Verified the live workspace was untouched: `diff -rq` against the mirror shows only the two intended files differ;
  `logs/` still holds only the two user files; the `__pycache__` seen in the source tree is dated July 30, i.e.
  not created this session.
- Stopped every process I started (checked with `ps`).
- Saved a memory note for future sessions.

---

## 4. Why each failure happened, in one table

| Failure | Why it happened | How it was found |
|---|---|---|
| Detector published z=0.43 / 0.695 | Mask accepted grey arm pixels; with the cube occluded the centroid and depth sample landed on the arm; no plausibility check | Saved a frame, applied the exact mask code, found grey pixels and a centroid outside the mask |
| 8 mm x error on a clean scene | Same leakage, smaller scale | Disappeared once the mask excluded the arm |
| Yaw hypothesis | Looked consistent with grasp widths | One quaternion read refuted it |
| Wall-clock hypothesis | Plausible mechanism | RTF samples inconclusive; later showed the sim is slow but headless runs still failed, so not causal |
| Table-point grasp fails | **Not resolved.** Finger 1 lags, width read from finger 1 only | Free-air traces |
| My shell kills (x2) | `pgrep/pkill -f` matched my own command text or a parent wrapper | Exit code 144 / vanished sim |
| `rclpy` import error | `python3 -I` ignores `PYTHONPATH` | Traceback |

---

## 5. Why the solution works (mechanism, not just "it passed")

1. **Colour test.** The cube is saturated blue; the arm is neutral or only slightly tinted. A test on *differences* between channels
   rejects every neutral pixel (difference 0) regardless of brightness, whereas the old test compared absolute
   levels, and absolute levels overlap (120 satisfies both a "high blue" and a "low red" bound).
   The margin is small (cube B-G=44 vs arm worst case 31), so the thresholds sit just under the cube's value;
   shading on the cube's side faces would lower B-G, which is acceptable only because the camera sees the top face.
2. **Median depth over the mask.** A centroid summarises *position*, not *surface*. Taking depth from the object's own
   pixels guarantees the depth belongs to the object (or to an outlier minority the median ignores).
3. **Plausibility gate.** Converts "confidently wrong" into "unknown". Downstream code (`resolve_object`) already
   handles "no pose" by refusing to move, so the failure becomes safe and visible.
4. **Home before resolve.** Removes the cause of the occlusion at the moment the camera is read, instead of only
   detecting it.
5. **Evidence it works [VERIFIED]:** offline replay (clean frame correct, occluded frame yields nothing); live reads within
   about 1.5 mm of ground truth at two positions; the previous z=0.43 symptom did not reappear in four patched cycles.

What the evidence does *not* prove: which of the two `move_to.py`/detector changes alone would have been sufficient in the
live cycle. Both were in place for every patched live run. The offline replay does isolate the detector change.

---

## 6. What was left unresolved at that point (superseded by section 9)

1. **Table-point grasp failure** (2 of 2 patched attempts failed; 2 of 2 attempts from the spawn point passed;
   one earlier *unpatched* attempt at the spawn point also failed, with the biased x). Cause of the slow finger 1 unknown.
2. **Width logic reads one finger.** Even if finger 1 is fixed, an off-centre grasp is better judged on the sum of both
   fingers (the cube width was recovered exactly as q1+q2=0.0302 in the trace).
3. **Sim speed.** The Gazebo GUI plus camera rendering takes RTF from about 0.99 to about 0.31. Wall-clock-based waits in `move_to.py` are
   fragile under that. A headless launch option would help; the launch file hard-codes the GUI.
4. **Not applied.** The patch exists only in the mirror. The live workspace still has the old detector.
5. **Not folded into launch.** `object_detector` is still started by hand; `panda_bringup.launch.py` does not start it.
6. **No handoff docs for S20/S21** existed before this; this log and `CONTINUATION_S21.md` are the only record.
7. **Segfault on exit** from `move_to` (MoveItPy teardown) happens on every run, patched or not. Harmless to results, unaddressed.
8. **Detector assumptions remain:** one blue cube, top-down camera, resting on the ground, yaw ignored. The gate hard-codes
   a ground-resting height.
9. **Two-consecutive-pass rule** is *not* met for the full cycle: the second consecutive cycle failed. Do not call the
   pipeline DONE.
10. The 4 mm hand-position overshoot in the descend (hand reported 0.354 for a 0.351 target) was noticed but not analysed.

### Recommended next steps, in order
1. Per-finger test: command both fingers, log commanded vs measured position for each, at several arm poses, in free air.
   Decide whether finger 1 is slow because of the controller gains, the gz_ros2_control joint setup, or contact.
2. Change grasp detection to use both fingers (sum, and per-finger difference as an off-centre indicator).
3. Re-run the two-consecutive-cycle test from **both** the spawn point and the `table` point.
4. Apply the patch to the live workspace (Khoi's decision), rebuild, add `object_detector` to the launch file.
5. Write the S20/S21 handoff.

---

## 7. General method (what to reuse on any perception-to-action bug)

### 7.1 Order of operations
1. Read existing logs and source before running anything. Check that the previous session's claims about what was or
   was not captured are true.
2. Separate the pipeline into stages and give each a **ground truth** (here: the simulator's true object pose).
   Compare *each stage's output* with its truth, not just the final outcome.
3. Test the earliest suspect stage **in isolation on a clean scene**. If it passes, the bug depends on state; find which state
   (here: arm position).
4. When a number is impossible, **save the raw sensor data** and run the exact production code on it offline.
   Look at the image. Print the pixel values the code accepted.
5. Ask "which single measurement would refute this hypothesis?" before building anything for it. Both wrong turns
   here (yaw, timing) were refuted or demoted in one cheap command.
6. Fix in a copy. Validate the fix **offline on saved data first**, then live. Offline is deterministic.
7. Re-run the project's own acceptance rule (here: two consecutive logged passing runs). A single success hides defects;
   in this session it hid two.
8. Keep a defect list separate from the fix list. Fixing one cause often exposes the next; do not assume the symptom is gone because the
   headline error changed.

### 7.2 Perception pitfalls worth checking in any colour-keyed detector
- Absolute thresholds on RGB overlap with neutral colours. Prefer channel *differences* or HSV with a saturation floor.
- Never take depth from a single pixel at a centroid. Use a robust statistic over the object's pixels.
- A centroid of an occluded or scattered mask can lie outside the mask. Log whether it does.
- Add a plausibility gate using known geometry and prefer publishing nothing to publishing a wrong value.
- Log the mask pixel count and bounding box. A bounding box far larger than the object is an immediate red flag.
- Perception that is read while the actuator is in the camera's view is unreliable; sequence "move out of view, then look".

### 7.3 Grasp-detection pitfalls
- Reading one joint of a multi-actuator gripper makes asymmetric grasps invisible. Read all of them.
- "Reached target exactly" and "stopped short" conditions depend on which finger you read; check that the assumption
  of symmetric closure is actually true in the simulator.
- Judge grasp width by the sum of finger positions against the known object size when possible.

### 7.4 Simulation hygiene
- Measure the real-time factor before trusting wall-clock timing. GUI plus camera rendering can cut it by two thirds.
- Prefer a headless server for automated tests; keep the GUI for manual inspection.
- Use the simulator's service to reset object poses (`/world/<name>/set_pose`) instead of restarting.
- Distinguish "slow" from "wrong": run the same test under a healthy RTF before blaming timing.

### 7.5 Shell and tooling hygiene
- Avoid `pkill -f` / `pgrep -f` with a pattern that appears in your own command line. Use PIDs from `ps | awk` or separate calls.
- Do not search `/` with `find`. Bound with `-maxdepth` and a directory.
- `python3 -I` ignores `PYTHONPATH`; do not use it for scripts that need ROS packages unless the paths are passed another way.
- Redirect every long-running node to a file at launch, and save the file *before* the next run overwrites state.
- Name logs for what they contain. (The handoff's `..._run1.log` was actually the failing run; this caused confusion.)
- Keep experimental scripts outside the project tree; when imports must come from the project, set `PYTHONDONTWRITEBYTECODE=1`.

### 7.6 Working under a read-only-source rule
- Source the install space (read-only use) to run the system; put new code in a mirror; run mirror scripts directly with `python3 <file>`
  so no build or install step touches the original.
- Verify at the end that the original is unchanged (`diff -rq` with the mirror excluding build dirs; check `logs/`; check timestamps of anything
  unexpected such as `__pycache__`).
- Hand the change back as a unified diff and let the owner apply it.

### 7.7 Reporting discipline
- Label every claim verified, inferred or untested.
- Report counts (2 of 2, 0 of 2), not adjectives.
- Say what a change was *not* shown to do (here: that each of the two patches was individually sufficient).
- Do not call a pipeline done when the project's own acceptance rule is unmet.

---

## 8. Reproduction material

All paths are on Khoi's machine. Scratch scripts are in `~/ros_test_ws_mirror/logs/s21_claude_code/`.

```bash
# 0. environment
source /opt/ros/jazzy/setup.bash && source ~/ros_test_ws/install/setup.bash
export GZ_VERSION=harmonic PYTHONDONTWRITEBYTECODE=1

# 1. bring up (GUI, as in the project)          -> ros2 launch twin_action_demo panda_bringup.launch.py
#    or headless (RTF ~1.0)                     -> ros2 launch ~/ros_test_ws_mirror/logs/s21_claude_code/bringup_headless.launch.py

# 2. spawn the cube
ros2 run ros_gz_sim create -world empty -name b -file ~/ros_test_ws/grasp_box.sdf -x 0.45 -y 0.0 -z 0.03

# 3. ground truth for the cube
gz topic -e -n 1 -t /world/empty/dynamic_pose/info | grep -A14 'name: "b"'

# 4. patched detector (mirror) and patched cycle (mirror)
python3 ~/ros_test_ws_mirror/src/twin_moveit_scripts/twin_moveit_scripts/object_detector.py 2>&1 | tee ~/ros_test_ws_mirror/logs/detector.log
python3 ~/ros_test_ws_mirror/src/twin_moveit_scripts/twin_moveit_scripts/move_to.py pick_and_place box table 2>&1 | tee ~/ros_test_ws_mirror/logs/run.log

# 5. reset between runs
ros2 run twin_moveit_scripts move_to grip 0.04
ros2 run twin_moveit_scripts move_to named ready
gz service -s /world/empty/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
   --req 'name: "b", position: {x: 0.45, y: 0.0, z: 0.016}, orientation: {w: 1}'

# 6. raw frame + mask diagnostic       -> python3 grab.py out.npz      (prints mask size, bbox, centroid, in-mask flag, depth stats)
# 7. finger time series                -> python3 closetrace.py        (open, then close; prints q1, q2 against time)
# 8. real-time factor                  -> gz topic -e -t /stats        (read real_time_factor)
```

### Evidence index (in `logs/s21_claude_code/`)
| File | What it shows |
|---|---|
| `detector.log` | unpatched detector, clean scene: (0.442, 0.002, 0.015) |
| `run1.log` | unpatched cycle: fails, `width=0.0283 without stalling` |
| `frame_poisoned.png` | the hand covering the cube; grey arm pixels fill the mask |
| `frame_ready.png` | clean frame at `ready`; cube visible |
| `detector_fixed.log`, `detector_fixed2.log` | patched detector reads (0.451, -0.000, 0.015) |
| `fixed_run1.log`, `fixed_run2.log` | patched cycles under the GUI sim: pass, then fail at the table point |
| `headless_run1.log`, `headless_run2.log` | patched cycles headless: pass, then the same failure |
| `bringup.log`, `bringup2.log` | launch output (GUI, headless) |
| `bringup_headless.launch.py` | scratch launch file with the GUI removed |
| `closetrace.py`, `grab.py` | diagnostic scripts |

### Key numbers
| Quantity | Value |
|---|---|
| Cube size | 0.03 m, rests at centre z=0.015 |
| Camera | world (0.4, 0, 1.2), looking down; cube top at depth 1.170 m |
| Cube pixel colour | (104, 172, 216): B-R 112, B-G 44 |
| Worst arm pixel in the old mask | B-G 31, B-R up to 155 (hand dots), depth < 1.1 m |
| Old detector on the failing state | z 0.43 / 0.57 / 0.695 against truth 0.015 |
| Patched detector error | about 1 mm (spawn), about 1.5 mm (table) |
| Healthy grasp widths | 0.0139, 0.0129 (S19: 0.012-0.015) |
| Failure signature | `width=0.0236` or `0.0283`, no stall |
| RTF | 0.31 with GUI (min 0.10), 0.99 headless |
| Finger 1 vs finger 2 speed | about 4-5 mm/s vs about 12-16 mm/s in free air (commanded: 16 mm/s) |

---

## 9. Completion: from "1 pass, 1 fail" to 12 consecutive passes

### 9.1 Why the work continued

Khoi asked why the errors were not all resolved. The honest answer: nothing had blocked the work. The
previous stop was a choice to end at a diagnosis. The read-only rule only blocks writes to `~/ros_test_ws`; everything
could be fixed and tested in the mirror. **Lesson: "I wrote it up as an open issue" is not the same as "I could not fix it".
State which one applies.**

### 9.2 Defect 2 root-caused: the width reading and the hold stroke

Recording **both** finger joints continuously through two full cycles (`rec_fingers.py`,
`fingers_cycles.csv`) showed what no single log line could [VERIFIED]:

- Cycle 1 close: both fingers about 16 mm/s, stopping at q1+q2 = 0.0302 (the cube width).
- Held cube: q1 and q2 see-saw by about +/-4 mm in opposite directions while **q1+q2 stays at 0.0302**.
- After the first grasp and release, some closes are asymmetric: q1 about 4.5 mm/s, q2 about 16 mm/s.
- Failing close: q2 swept to 0.0019 while q1 was at 0.0293. Sum 0.0312: **the cube was in the hand**, about 13 mm off-centre.

The failure chain was then plain:
1. `FINGER_JOINT` made `width` mean finger 1 only, so the detect stroke read 0.0291.
2. The hold stroke commanded **both** fingers to 0.0291 - 0.005 = 0.0241.
3. That drove finger 2 *outward* from 0.002 to 0.024, letting go of the cube.
4. Finger 1 reached the target exactly, so the check said "nothing squeezed, stall was spurious".
5. `pick()` reported "no object between the fingers" about a grasp the code had just released.

**Fix:** `width` is now the **mean of both fingers**. Units are unchanged, so no threshold moved. All readers go through one callback
(`_on_joint_states`), so the fix is one small change. The mean does not move during the see-saw.
Result: grasp width went from a scatter (S19: 0.0123 / 0.0140 / 0.0151; this session 0.0129, 0.0139) to **0.0151 on every full-success cycle of the final run.**
This also likely explains S19's "grasp width varies run-to-run" note [INFERRED].

### 9.3 Trap: logs said success, ground truth said failure

After 3 passes from the spawn point, I tested a harder start (cube at (0.5, -0.12), yawed 30 degrees). All 3 cycles logged
`pick=True, place=True`. **Ground truth showed the cube never reached the target.** The log showed `grasp at 0.0365`: the hand closed
on nothing, the hold stroke met some resistance (cause not established), the code called that "compression", lifted an empty hand, and "placed" nothing.
Only printing the simulator's pose after every cycle exposed this. **Had the check stopped at the log line, this would have been reported as 6 for 6.**

**Fix:** size plausibility in `pick()`. A stall must be within `GRASP_WIDTH_TOL` (0.006) of `CUBE_HALF_WIDTH` (0.015), otherwise it fails with
`grasp_width_implausible` and does not lift. In the final run this check fired exactly once, correctly (9.6).

### 9.4 Fingertips on the ground

`finger.stl` measures z 0.0001 to 0.0538, and the finger frame is 0.0584 below link8, so fingertips are about 0.112 below link8 [VERIFIED].
At `FINGER_REACH_Z = 0.093` the commanded z=0.108 puts tips about 4 mm *below* the ground, and the arm stopped at 0.112 on every run: it was being
blocked by the floor. With 0.100 it reaches its 0.115 command [VERIFIED].

- Direct comparison at (0.5, -0.12), yaw 0: **both** 0.093 and 0.100 passed, one run each. So this height change was **not shown to be necessary**.
- It was kept because pressing into the floor is plainly unwanted and 0.100 passed everything. Its comment in the source records why.
- It contradicts an old source comment saying 0.10 grasped nothing (S17). That result predates the S19 object-height fix, which may be why [INFERRED].
- Whether ground contact caused the slow finger was **not established**. The per-close speed analysis was too noisy to say.

### 9.5 Defect 3: `ok=True` for a move that never happened

During an extended batch one cycle failed with `no_object_resolved`. The detector was right: the hand was still over the cube.
The log showed why: MoveIt cancelled the move to `ready` 50 ms after it started (`TIMED_OUT` against a 4.8 s bound), yet the code logged
`move_to_named('ready') -> ok @ (0.350, 0.200, 0.265)`, which is not the ready pose. `_plan_execute` never checked the outcome of `robot.execute`.

**Fix:** `_plan_execute` compares the live end-effector position with the plan's last waypoint (tolerance `ARRIVAL_TOL`, 0.02 m), replans once
from the live state if the arm missed, and otherwise returns the new stage `exec_incomplete`.

Tested two ways [VERIFIED]:
- Fault injection (`fault_inject.py`, wrapping `robot.execute`): one dropped execution recovers, two give `exec_incomplete`, none is unaffected.
- The real fault twice in the final run, on `newstart1` (watchdog `TIMED_OUT`) and `newstart2` (`Action client not connected`, `ABORTED`). Both cycles still passed.

Both real occurrences were the **first move after a new process started**, which suggests a start-up race [INFERRED]. The retry handles the symptom; the race itself is not fixed.

### 9.6 Final acceptance (fresh sim, final code, ground-truth verdicts)

Evidence: `logs/s21_claude_code/acceptance/accept2.out`. A cycle counted only if the code reported success **and** the simulator showed the
cube within 2 cm of the target. Headless sim. Patched scripts run directly from the mirror source.

| Batch | Start | Result | Retries |
|---|---|---|---|
| `spawn1-8` | spawn point, then table to table, no resets | **8 of 8** | 0 |
| `newstart1-4` | cube reset to (0.5, -0.12) each time | **4 of 4** | 2 |
| `yaw30_1-3` | cube yawed 30 degrees | **2 of 3**; 1 honest failure | 0 |

- **12 of 12 consecutive** for axis-aligned cubes, exceeding the 3 asked for and the project's rule of 2.
- Every cycle reporting success matched ground truth (14 of 14). No false positives.
- Placement landed within 2.2 mm in x and 5.0 mm in y of the target for the 12 aligned cycles; all at z=0.015.
- `yaw30_2` failed honestly with `grasp_width_implausible` (width 0.0275) and did not lift.
- `yaw30_3` is a weak sample: after the previous failure I teleported the cube into the still-closed hand and it was knocked to about (0.444, 0.116). The pipeline perceived and picked it there, which is useful, but its yaw was no longer the 30 degrees set.

### 9.7 Everything changed (all in `s21_detector_fixes.diff`, 332 lines; verified to reproduce the mirror exactly on an untouched copy)

| File | Change | Why |
|---|---|---|
| `object_detector.py` | Blue-dominance colour test | Grey arm pixels passed the old test |
| `object_detector.py` | Median depth over mask pixels | The centroid can land off the object |
| `object_detector.py` | Plausibility gate on z | Publish nothing rather than a wrong pose |
| `move_to.py` | Home before `resolve_object` in `pick()` | The arm hid the cube from the camera |
| `move_to.py` | `width` = mean of both fingers | One finger measured the see-saw, not the grip |
| `move_to.py` | `grasp_width_implausible` check | Stopped false grasps of empty air |
| `move_to.py` | Arrival check and one retry | `ok=True` was reported for cancelled moves |
| `move_to.py` | `FINGER_REACH_Z` 0.093 to 0.100 | Tips pressed into the floor |

### 9.8 What is still unresolved

1. **Not applied.** `~/ros_test_ws` still has the old code. Apply with:
   `cd ~/ros_test_ws/src/twin_moveit_scripts/twin_moveit_scripts && patch -p0 < ~/ros_test_ws_mirror/s21_detector_fixes.diff`
   then `cd ~/ros_test_ws && colcon build --packages-select twin_moveit_scripts`.
2. **The installed package path is untested.** All runs executed the scripts from mirror source, not through `ros2 run` after a build. Re-run 3 cycles after applying.
3. **The GUI path is untested for the final code.** The final run was headless. The GUI gave RTF 0.31 here and `move_to` uses wall-clock waits, so test it. Earlier GUI runs did show the same pass/fail pattern as headless.
4. **Yaw is not handled.** The detector reports position only. A 30 degree cube gave 2 of 3 (one weak sample), failing honestly. Real support needs yaw estimation and hand rotation.
5. **Why finger 1 is sometimes slow is unknown.** It is now tolerated, not fixed. Unexplored lead: the URDF gives `panda_rightfinger` mass **0.15** and `panda_leftfinger` **0.015**, a likely typo, not changed or tested. Fingers also stayed frozen at the arm's spawn pose in some tests; not explained, and it did not affect cycles because `pick()` homes first.
6. **Start-up race in MoveIt execution** (9.5) is retried, not cured.
7. **Colour margin is thin:** cube B-G 44 against arm worst case 31, threshold 36, and tuned to this sim's lighting.
8. **`object_detector` is still not in the launch file**, and the launch file hard-codes the GUI.
9. **Segfault on exit** from `move_to` on every run. Results are unaffected.
10. **The plausibility check and z-gate hard-code the 3 cm cube on the ground.** A different object needs those constants changed.
11. **The LLM path (`llm_move.py`) was not re-tested** with these changes. `move_to.py`'s callers get the new failure stages `grasp_width_implausible` and `exec_incomplete`.

### 9.9 Additions to the general method (section 7)

- **Verify outcomes in the world, not in the program's own report.** Every status here lied at least once: JTC's "Goal successfully reached" with frozen joints, `HELD` on an empty hand, `place=True` with an unmoved cube, `ok` on a cancelled move. Use an independent source, and make it the pass mark.
- **Record all channels continuously through the whole scenario.** Snapshots hid what a time series of both fingers showed at once.
- **Harden the test after it passes.** The false positive and the watchdog bug only appeared when the test was made harder and longer. Three passes from one start proved less than it looked.
- **Check derived quantities for invariants.** q1+q2 was constant while q1 and q2 swung; finding what stays constant reveals the right measurement.
- **When an invariant is physically meaningful, assert it.** A known object size turns "stopped short" into a real test.
- **Inject faults for rare failure paths.** Do not wait for a 1-in-20 event to test its handler.
- **Any code change voids earlier results.** Rerun acceptance on the final code on a fresh system.
- **Fix the reporting first when a safe refusal looks like a bug.** In 9.5 the detector's refusal was right; the bug was upstream.
- **Say whether you stopped because you were blocked or because you chose to.**
