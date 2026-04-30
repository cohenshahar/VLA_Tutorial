# VLATraining — Agent Instructions

**Project:** BGU Mechatronics MSc thesis — VLA Simulation Environment
**Author:** Shahar Cohen | **Advisor:** Prof. Amir Shapira
**Date:** 2026-04-28

---

## What this project is

A full MuJoCo simulation of a KUKA KR AGILUS KR 6 R900 sixx arm performing electromagnetic pick-and-place, used to train and evaluate an OpenVLA policy. All code runs headlessly on Kaggle / Google Colab GPU (T4 minimum, A100 preferred).

See [sim_work_plan.md](../sim_work_plan.md) for the full milestone plan and "Done when" acceptance criteria.
See [sim_instructions.md](../sim_instructions.md) for step-by-step task breakdowns.
See [sim_block_diagram.mermaid](../sim_block_diagram.mermaid) for the full architecture diagram.

---

## Hard rules (always enforce)

- **One file per tool call.** Never create or edit more than one file in a single response.
- **No human in the loop.** Every state transition must be autonomous — no `input()`, no interactive prompts.
- **No code before approval.** Do not start a milestone until Shahar confirms the plan for that milestone.
- **Context window rule.** When approximately 60% of the context window is consumed, warn Shahar to continue in a new conversation. Before closing the conversation, produce a handoff summary covering: work completed, current task/milestone, next steps, and any open questions.

---

## Fail sequence (always follow when a task is stuck)

**Trigger:** 3 failed attempts on the same step.

**Sequence:**
1. On attempts 1 and 2 — try one alternative approach before retrying (change the strategy, not just re-run the same command).
2. On attempt 3 failure — **stop immediately** and report using the format below. Do NOT continue to the next task.
3. Wait for Shahar to provide a fix or direction, then apply it exactly and retry the task from the beginning.

**Report format (mandatory on failure):**
```
FAIL — Task <number>: <name>
Tried:
  1. <first approach + error>
  2. <second approach + error>
  3. <third approach + error>
Suggested next steps:
  - <option A>
  - <option B>
Waiting for your instruction before continuing.
```

**Resume rule:** Only continue after Shahar explicitly says what to fix. Apply the fix, then retry the failed task from scratch.

---

## Project structure

```
VLATraining/
  AGENTS.md               ← this file
  .gitignore
  .github/
    prompts/              ← slash-command prompts (/run-milestone, /validate-mjcf, /verify-sensors)
    instructions/         ← file-scoped coding conventions
  sim/
    assets/
      urdf/               ← KUKA xacro + converted kr6r900sixx.urdf
      mjcf/               ← generated MJCF files
    arm/                  ← load_arm.py and arm utilities
    scene/                ← world.xml, test_world.py, test_joints.py
    bridge/               ← SimBus or ROS2 bridge
    task_tree/            ← TaskNode, TaskTreeManager
    outputs/              ← PNG renders, MP4 videos, CSV logs (gitignored media)
```

Python venv: `phase4_env/` (workspace root, already activated — use `source /home/shahar/Desktop/phase4/phase4_env/bin/activate`)

---

## Robot — KUKA KR AGILUS KR 6 R900 sixx

| Parameter | Value |
|---|---|
| Payload | 6 kg |
| Reach | 901 mm |
| Joints | 6 revolute: **A1–A6** |
| Repeatability | ±0.03 mm |
| Joint limits | A1 ±170°, A2 −190°/+45°, A3 −120°/+156°, A4 ±185°, A5 ±120°, A6 ±350° |
| Controller | KR C4 |

**URDF source:** `ros-industrial/kuka_experimental` → `kuka_kr6_support/urdf/kr6r900sixx.xacro`
Sparse-cloned to: `sim/assets/urdf/kuka_experimental/kuka_kr6_support/`
Standalone xacro (no ROS package deps): `sim/assets/urdf/kr6r900sixx_standalone.xacro`
Converted URDF: `sim/assets/urdf/kr6r900sixx.urdf` ✅ verified nq=6, all limits match datasheet

**URDF loading notes (verified MuJoCo 3.6.0):**
- The `<mujoco><visual><global offwidth/offheight>` tag in URDF is NOT picked up by MuJoCo at runtime.
  Fix: set `model.vis.global_.offwidth` and `model.vis.global_.offheight` in Python before creating `mujoco.Renderer`.
- MuJoCo resolves URDF mesh paths relative to the URDF file's directory using basename only.
  Fix: add `<mujoco><compiler meshdir="meshes/"/></mujoco>` in URDF AND place all mesh files flat in `assets/urdf/meshes/`.
- `kuka_resources` is a separate ROS package (not in `kuka_experimental`). Use `kr6r900sixx_standalone.xacro` which inlines the material stubs.
- MuJoCo 3.x does NOT have `mujoco.utils.find_base_path()` — use `gymnasium` or `dm_control` assets for test models.

**Joint names in URDF:** `joint_a1` … `joint_a6` (revolute), plus `joint_a6-flange`, `base_link-base`, `flange-tool0` (fixed).

---

## Stack

| Component | Choice |
|---|---|
| Simulator | MuJoCo (`pip install mujoco`) — headless, `mjr_render` + `mjr_readPixels` for cameras |
| ROS2 | ROS2 Humble — test in Colab first; fall back to SimBus if install fails |
| Message bus fallback | `SimBus` — pure-Python `dict[str, queue.Queue]`, zero dependencies |
| VLA model | `openvla/openvla-7b` (HuggingFace) — wrist RGB 224×224 + language instruction → 7-DoF Δ joints |
| Action normalization | BridgeData v2 stats (default); override with KUKA-specific stats if fine-tuning |
| Physics timestep | 0.001 s (1000 Hz); bus publish rates: joints 100 Hz, cameras 6 Hz |

---

## Milestone map (M1–M10)

| # | Name | Key artefact |
|---|---|---|
| M1 | KUKA URDF → MJCF | `outputs/arm_home_pose.png`, `model.nq == 6` |
| M2 | Joint control + realistic physics | Torque log, no limit violations |
| M3 | Full pick-and-place scene | Box lifts via EM, drops on deactivate |
| M4 | Sensor suite | All 6 topics at correct rates |
| M5 | Multi-camera pipeline | 3 PNGs, combined render ≥6 Hz |
| M6 | ROS2 / SimBus bridge | All 10 topics active |
| M7 | Task Tree Manager | 3-subtask sequence driven by scripted policy |
| M8 | OpenVLA integration | 7-element action array in ≤2 s |
| M9 | Full closed-loop execution | ≥2/3 pick-and-place attempts succeed |
| M10 | Baseline logging & replay | Determinism test passes, Matplotlib animation works |

Full specs and "Done when" conditions: [sim_work_plan.md](../sim_work_plan.md)

---

## Key conventions

### Joint naming
Always use the canonical joint names `A1`–`A6` exactly as they appear in the URDF. Never abbreviate or renumber.

### EM end-effector (weld equality constraint)
```python
# Activate
model.eq_active[em_weld_id] = True   # or 1
# Deactivate
model.eq_active[em_weld_id] = False  # or 0
```
Pre-condition: proximity sensor < 0.02 m AND magnet flag ON before activating.

### Sensor indexing
Use named lookup rather than hard-coded indices:
```python
sensor_id = model.sensor(name).id
value = data.sensordata[sensor_id]
```

### Camera rendering (offscreen)
```python
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera="camera_name")
pixels = renderer.render()   # numpy uint8 [H, W, 3]
```

### SimBus topic names (canonical)
```
/joint_states            100 Hz   shape: (12,) = qpos[6] + qvel[6]
/joint_torques           100 Hz   shape: (6,)
/joint_commands          on-demand shape: (6,) target qpos
/ft_sensor               100 Hz   shape: (6,) = force[3] + torque[3]
/proximity               50 Hz    shape: (1,) metres
/em_state                on-change bool
/target_contact          50 Hz    bool
/camera/wrist/image_raw  6 Hz     uint8 [480, 640, 3]
/camera/overhead/image_raw 6 Hz   uint8 [480, 640, 3]
/camera/side/image_raw   6 Hz     uint8 [480, 640, 3]
/task_tree/status        on-change JSON string
```

### F/T acceptance threshold
Box mass 0.5 kg → expected vertical force during EM hold: **≥ 4.5 N** (0.5 × 9.81 with margin).

### Output file naming
All renders → `sim/outputs/<descriptive_name>.png`
All videos → `sim/outputs/<descriptive_name>.mp4`
All logs   → `sim/outputs/<run_id>_<component>.csv`

---

## Environment setup (quick reference)

```bash
# Activate venv (already done if terminal shows (phase4_env))
source /home/shahar/Desktop/phase4/phase4_env/bin/activate

# Source ROS2 (if using ROS2 option)
source /opt/ros/humble/setup.bash

# Verify MuJoCo
python -c "import mujoco; print(mujoco.__version__)"

# Clone KUKA URDF (M1)
cd sim/assets/urdf
git clone --depth=1 https://github.com/ros-industrial/kuka_experimental
```

---

## What is NOT in scope (Phase 1)

Do not build: TimesFM observer, Semantic Verifier, Emergency Planner, failure injection harness, evaluation metrics. These are Phase 2.
