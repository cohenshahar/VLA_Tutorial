# CLAUDE.md — VLA_Tutorial

**Last updated:** 2026-05-11
**Repo:** cohenshahar/VLA_Tutorial
**Owner:** Shahar Cohen (shaharag@post.bgu.ac.il)

This file is read by Claude Code at the start of every session in this repo.
Read it end-to-end before writing any code.

---

## 1. What this project is

A **tools-and-bounds simulator** for a 6-DOF robot arm with an electromagnetic gripper.
Goal: build a simulation platform that can run existing open-source VLA (Vision-Language-Action) models and characterize what the simulator can deliver.

This is NOT thesis content. Do not write about the Verifier, Emergency Planner, or novelty checks here.
The thesis lives in a separate folder (`VLA cowork/`). Keep them separate.

---

## 2. Stack

| Layer | Tech |
|-------|------|
| Physics | MuJoCo 3.8 |
| ROS2 bridge | ROS2 Humble — `VLATraining/vla_ws/` |
| Bridge node | `VLATraining/vla_ws/src/mujoco_bridge/mujoco_bridge/bridge_node.py` |
| Sim code | `VLATraining/sim/` |
| Python | 3.10, Ubuntu 22.04 |
| GPU inference | Kaggle (deferred — use only when needed) |

**Measured simulator bounds (Phase 9):**
- RTF with cameras: 0.756 (3 × 6 Hz cameras, CPU osmesa)
- RTF without cameras: 8.843
- High-freq topic delivery: ~75–85 Hz (GIL-bound, spec is 100 Hz)
- Camera topics: 6 Hz ✅

---

## 3. Directory map

```
VLATraining/
├── sim/
│   ├── scene/              ← world XML, _ik_utils, em_controller, sensor_logger
│   │   ├── _ik_utils.py    ← shared DLS IK (Phase 10)
│   │   ├── demo_reach.py          ← Task 10.1
│   │   ├── demo_grasp_lift.py     ← Task 10.2  (--object box|sphere)
│   │   ├── demo_place_near.py     ← Task 10.3
│   │   ├── demo_place_far.py      ← Task 10.4
│   │   └── demo_full_sequence.py  ← Task 10.5
│   ├── arm/                ← kinematics, load_arm
│   ├── task_tree/          ← Phase 10 target (currently empty)
│   ├── tools/              ← VLA pipeline + measurement scripts
│   │   ├── obs_pipeline.py        ← Task 10.7
│   │   ├── octo_adapter.py        ← Task 10.8
│   │   ├── mock_vla_loop.py       ← Task 10.9 (requires ROS2 bridge)
│   │   └── measure_topic_rates.py ← Phase 9 verification
│   ├── docs/               ← execution guides
│   ├── outputs/            ← logs, videos, measurements (committed)
│   └── SESSION_LOG.md      ← append a new entry every session
├── vla_ws/
│   └── src/mujoco_bridge/  ← ROS2 bridge node
PHASES.md                   ← phase tracker (update when phase completes)
```

---

## 4. Git rules (follow exactly)

1. **Commit after every completed task** — never batch multiple tasks into one commit.
2. **Commit message format:** `Phase 10, Task X.Y: <one line what changed>`
3. **Before committing:** `git status` — only stage files related to the current task.
4. **After every commit:** `git push origin main`
5. **Never:** force push, `--no-verify`, commit `.env` files or secrets.
6. **SESSION_LOG.md:** append a new section at the end of every session before the final commit.

---

## 5. Code rules

1. **One file per logical unit.** Do not put multiple unrelated classes in one file.
2. **Every script runnable standalone** — `if __name__ == '__main__':` with a clear main().
3. **No hardcoded paths.** Use `Path(__file__).resolve().parent` or env vars.
4. **Docstring on every class and non-trivial function.**
5. **After writing a script:** run `python3 -m py_compile <file>` to verify syntax.
6. **Test with real data where possible,** mock data where not (no ROS2 dependency in unit tests).

---

## 6. Current phase: Phase 10 — VLA-ready simulator

**Goal:** demonstrate the robot can perform basic manipulation primitives reliably,
build the observation/action adapters for VLA integration,
and run a full closed-loop test with the Octo model (on Kaggle when needed).

**Target VLA model:** Octo (93M params, open source, HuggingFace)
- Input: RGB image 256×256 + language instruction string
- Output: 7 floats — joint deltas for joints 1–6 + gripper state (0.0 or 1.0)
- Inference frequency: 6 Hz

**Object shapes to support:** metal box (default), sphere (--object sphere flag).
Grasp detection is shape-agnostic (proximity sensor only — no site-specific check).

---

## 7. Phase 10 task list

Complete tasks in order. Do not skip.

### Group A — Scripted primitives (no GPU needed)

**Task 10.1 — `sim/scene/demo_reach.py`** ← CODEBASE READY (written in Cowork)
Standalone MuJoCo script. Arm moves from home pose to hover above the metal box.
Done when: proximity sensor reads < 0.025 m. Print proximity at each step.
Output: `outputs/demo_reach_<date>.mp4` + sensor log printed to stdout.
**TODO on Linux:** run `python3 -m py_compile demo_reach.py`, then execute and verify.

**Task 10.2 — `sim/scene/demo_grasp_lift.py`** ← CODEBASE READY
Arm reaches box, EM activates, lifts 15 cm. Uses `em_controller.em_activate()`.
Done when: FT sensor reads ee_fz > 4.0 N for 100 consecutive steps after lift.
Output: `outputs/demo_grasp_lift_<date>.mp4`.
Flags: `--object box` (default) or `--object sphere` (Task 10.6).

**Task 10.3 — `sim/scene/demo_place_near.py`** ← CODEBASE READY
After lift, arm moves to above target zone at 2 cm height, EM deactivates.
Done when: target_contact == True within 1.0 s (1000 steps) of EM deactivation.
Output: `outputs/demo_place_near_<date>.mp4`.

**Task 10.4 — `sim/scene/demo_place_far.py`** ← CODEBASE READY
After lift, arm moves to above target zone at 20 cm height, EM deactivates, box drops.
Done when: target_contact == True within 2.0 s (2000 steps) of EM deactivation.
Output: `outputs/demo_place_far_<date>.mp4`.

**Task 10.5 — `sim/scene/demo_full_sequence.py`** ← CODEBASE READY
Combines Tasks 10.1–10.3 into a single script: reach → grasp → lift → transport → place near.
Renders wrist + overhead camera side by side with text overlay:
current step name, proximity, ee_fz, em_state, target_contact.
Output: `outputs/demo_full_sequence_<date>.mp4`.

### Group B — Shape flexibility (no GPU needed)

**Task 10.6 — Add second object to scene** ← CODEBASE READY
Sphere added to `scene/includes/objects.xml` at [0.55, 0.15, 0.88].
`em_controller.em_can_activate()` now uses proximity sensor (shape-agnostic).
`demo_grasp_lift.py --object sphere` activates sphere grasping.
**TODO on Linux:** verify objects.xml loads without error, run grasp_lift with both objects.

### Group C — VLA pipeline (no GPU needed)

**Task 10.7 — `sim/tools/obs_pipeline.py`** ← CODEBASE READY
Standalone test: `python3 obs_pipeline.py` — prints shape and dtype, no ROS2 needed.

**Task 10.8 — `sim/tools/octo_adapter.py`** ← CODEBASE READY
Standalone test: `python3 octo_adapter.py` — tests with known input/output.

**Task 10.9 — `sim/tools/mock_vla_loop.py`** ← CODEBASE READY
Closed-loop mock without real model: reads `/camera/wrist/image_raw` from ROS2,
passes through ObsPipeline, generates random 7-float action, passes through OctoAdapter,
publishes to `/joint_commands` at 6 Hz.
Done when: loop runs for 30 s without crash, bridge processes all commands,
`ros2 topic hz /joint_commands` shows ~6 Hz.
**Requires bridge running.** Startup check prints error if bridge not alive.

### Group D — Octo real inference (Kaggle GPU — deferred)

**Task 10.10 — Kaggle notebook**
Load Octo from HuggingFace. Accept observation via HTTP (ngrok).
Return 7-float action. Document the ngrok URL setup.

**Task 10.11 — Full VLA run**
Replace mock_vla_loop.py random actions with real Octo inference from Kaggle.
Record full pick-and-place attempt. Save video.
Document: how far did the model get? What failed?

---

## 8. Linux startup sequence for Phase 10

Run these on the Linux machine before every Claude Code session:

```bash
# 1. Pull latest (includes new Phase 10 files)
cd ~/Desktop/VLA_Tutorial
git pull

# 2. Verify Phase 10 files landed
ls VLATraining/sim/scene/demo_*.py
ls VLATraining/sim/tools/

# 3. Syntax-check all new files
for f in VLATraining/sim/scene/demo_*.py \
          VLATraining/sim/scene/_ik_utils.py \
          VLATraining/sim/tools/obs_pipeline.py \
          VLATraining/sim/tools/octo_adapter.py; do
  echo -n "$f : "
  python3 -m py_compile "$f" && echo "OK" || echo "SYNTAX ERROR"
done

# 4. Standalone tests (no ROS2, no GPU)
cd ~/Desktop/VLA_Tutorial
PYTHONPATH=VLATraining/sim python3 VLATraining/sim/tools/obs_pipeline.py
PYTHONPATH=VLATraining/sim python3 VLATraining/sim/tools/octo_adapter.py

# 5. Run Task 10.1 (first demo)
PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_reach.py
```

---

## 9. SESSION_LOG format

Append to `VLATraining/sim/SESSION_LOG.md` before the final commit of each session:

```markdown
## Session <YYYY-MM-DD> — Phase 10, Tasks X.Y–X.Z

### What changed
- file: description

### Test results
- Task X.Y: ✅ / ❌ — one line result

### Next session entry point
Task X.Z+1 — <what to do>
```

---

## 10. What to do when you open this repo cold

1. Read this file end-to-end.
2. Run `git log --oneline -5` — see where we are.
3. Read the last entry in `VLATraining/sim/SESSION_LOG.md`.
4. Read `PHASES.md` — check Phase 10 status.
5. Continue from the next incomplete task in §7 above.
6. Never start a task without reading its Done criteria first.

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-11*
