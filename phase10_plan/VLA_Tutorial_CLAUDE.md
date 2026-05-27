# CLAUDE.md — VLA_Tutorial

**Last updated:** 2026-05-27
**Repo:** cohenshahar/VLA_Tutorial
**Owner:** Shahar Cohen (shaharag@post.bgu.ac.il)

This file is read by Claude Code at the start of every session in this repo.
Read it end-to-end before writing any code.

---

## 1. What this project is

A **tools-and-bounds simulator** for a 6-DOF robot arm with an electromagnetic gripper.
Goal: build a simulation platform that can run existing open-source VLA (Vision-Language-Action)
models and characterize what the simulator can deliver.

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
| GPU | Always available — use freely |

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
│   ├── scene/              ← world XML, camera_utils, em_controller, sensor_logger
│   │   └── includes/       ← arm.xml, table.xml, objects.xml, cage.xml, lights.xml
│   ├── arm/                ← kinematics, load_arm
│   ├── task_tree/          ← Phase 10 active — Task Tree Manager lives here
│   ├── tools/              ← measurement + adapter scripts
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
build a Task Tree Manager that feeds VLA-compatible language instructions and
monitors sensor-based postconditions, and close the loop with the Octo model.

**Target VLA model:** Octo (93M params, open source, HuggingFace)
- Input: RGB image 256×256 + language instruction string
- Output: 7 floats — joint deltas for joints 1–6 + gripper state (0.0 or 1.0)
- Inference frequency: 6 Hz
- The Task Tree Manager is invisible to the VLA — it only feeds the current instruction

**Object in scope:** metal_box (cubic, 10×10×10 cm) only.
Sphere support already in em_controller.py — do not remove, but do not extend it now.

---

## 7. Phase 10 task list

Complete tasks in order. Do not skip. Check SESSION_LOG for latest status before starting.

---

### Group A — Scripted primitives ✅ DONE

| Task | File | Status |
|------|------|--------|
| 10.1 demo_reach | `scene/demo_reach.py` | ✅ |
| 10.2 demo_grasp_lift | `scene/demo_grasp_lift.py` | ✅ |
| 10.3 demo_place_near | `scene/demo_place_near.py` | ⚠️ verify after target zone fix |
| 10.4 demo_place_far | `scene/demo_place_far.py` | ⚠️ verify after target zone fix |
| 10.5 demo_full_sequence | `scene/demo_full_sequence.py` | ⚠️ verify after target zone fix |

---

### ⚠️ PREREQUISITE — Fix target zone position

**Do this before running 10.3, 10.4, 10.5, or any task tree task.**

The current target zone is at `pos="0 0 0.86"` in `scene/includes/objects.xml`.
That puts it only 0.3 m from the arm base (arm base at [0, −0.3]) — inside the arm's minimum reach.
The arm cannot approach it top-down.

**Fix:** change `target_zone` body pos to `"0.5 0.4 0.86"` in `scene/includes/objects.xml`.

Verify: `demo_place_near.py` reports `target_contact == True` before attempting task tree work.
Also update `TARGET_XY = np.array([0.5, 0.4])` in `demo_place_near.py`, `demo_place_far.py`,
and `demo_full_sequence.py`.

Commit message: `Phase 10, prereq: fix target zone to [0.5, 0.4] — arm reachable`

---

### Group B — Shape flexibility (sphere already in em_controller — skip for now)

Task 10.6 — deferred. Sphere support already exists in `em_controller.py`. Do not touch.

---

### Group C — VLA pipeline adapters

**Task 10.7 — `sim/tools/obs_pipeline.py`**
Class `ObsPipeline` with method `get_observation(image_bgr, instruction: str) -> dict`.
Takes a BGR numpy array (any size), resizes to 256×256 RGB, normalizes to float32 [0,1],
returns `{"image": np.ndarray shape (256,256,3), "instruction": str}`.
Standalone test: `python3 obs_pipeline.py` — prints shape and dtype, no ROS2 needed.

**Task 10.8 — `sim/tools/octo_adapter.py`**
Class `OctoAdapter` with method `actions_to_joint_cmd(actions: np.ndarray) -> dict`.
Input: 7 floats from Octo (joint deltas 0–5 + gripper 6).
Output: dict with `joint_positions` (list of 6 floats, clipped to joint limits)
and `em_command` (bool: gripper > 0.5 → True).
Joint limits: all joints ±2.8 rad.
Standalone test: `python3 octo_adapter.py` — tests with known input/output.

**Task 10.9 — `sim/tools/mock_vla_loop.py`**
Closed-loop mock without real model: reads `/camera/wrist/image_raw` from ROS2,
passes through ObsPipeline, generates random 7-float action, passes through OctoAdapter,
publishes to `/joint_commands` at 6 Hz.
Done when: loop runs for 30 s without crash, `ros2 topic hz /joint_commands` shows ~6 Hz.
Requires bridge running. Add startup check that prints error if bridge not alive.

---

### Group D — Task Tree Manager ← ACTIVE WORK

**Context:** The Task Tree Manager is the meta-controller that sits above the VLA.
It manages which subtask is currently active, feeds the VLA a single language instruction
per subtask, and uses live sensor data to evaluate postconditions and advance the tree.
The VLA never sees the tree — it only receives the current instruction + camera image.

**The 7-node tree for metal_box pick-and-place:**

```
Task: EM Pick-and-Place (metal_box)
├── 1. SEARCH        → "locate the metal box on the table"
├── 2. APPROACH      → "move above the metal box"
├── 3. GRASP         → "grasp the metal box"
├── 4. LIFT          → "lift the metal box up"
├── 5. LOCATE TARGET → "find the red target zone"
├── 6. TRANSPORT     → "move the box to the red target zone"
└── 7. PLACE         → "release the metal box onto the target zone"
```

**Postconditions (sensor sources → pass criteria):**

| Node | Sources | Pass when |
|------|---------|-----------|
| SEARCH | overhead_cam, box_pos | box_pos is finite (always true in sim) |
| APPROACH | proximity, wrist_cam | proximity < 0.025 m |
| GRASP | em_active, ee_force | em_active == True AND ‖ee_force‖ > 4.0 N |
| LIFT | ee_force, box_pos.z | ‖ee_force‖ > 4.0 N AND box_pos.z > TABLE_Z + 0.15 |
| LOCATE TARGET | overhead_cam, target_pos | target_pos is finite (always true in sim) |
| TRANSPORT | ee_pos, ee_force | ‖ee_xy − target_xy‖ < 0.08 m AND box_z > TABLE_Z + 0.08 |
| PLACE | target_contact, em_active | target_contact > 0.0 AND em_active == False |

**Scene constants used by postconditions:**
```python
TABLE_Z    = 0.85   # m
TARGET_XY  = np.array([0.5, 0.4])  # m — after target zone fix
```

---

**Task 10.10 — `sim/task_tree/sensor_reading.py`**

Dataclass `SensorReading` — all sensor state in one flat object.
No MuJoCo imports — pure Python dataclass.

Fields:
```python
@dataclass
class SensorReading:
    t:              float           # simulation time (s)
    qpos:           np.ndarray      # shape (6,) — joint positions (rad)
    qvel:           np.ndarray      # shape (6,) — joint velocities (rad/s)
    torque:         np.ndarray      # shape (6,) — actuator forces (N·m)
    ee_force:       np.ndarray      # shape (3,) — [fx, fy, fz] at em_contact_site (N)
    ee_torque:      np.ndarray      # shape (3,) — [tx, ty, tz] at em_contact_site (N·m)
    proximity:      float           # rangefinder (m); −1.0 = no hit
    em_active:      bool            # weld constraint active?
    target_contact: float           # target_touch_sensor value (N)
    overhead_bgr:   np.ndarray      # shape (H, W, 3) uint8 — may be None
    wrist_bgr:      np.ndarray      # shape (H, W, 3) uint8 — may be None
    box_pos:        np.ndarray      # shape (3,) world position of metal_box body
    ee_pos:         np.ndarray      # shape (3,) world position of em_contact_site
```

Include a factory classmethod:
```python
@classmethod
def from_mujoco(cls, model, data, cam_publisher=None) -> "SensorReading":
```
Reads all fields from MuJoCo model/data directly.
`cam_publisher` is optional — if None, `overhead_bgr` and `wrist_bgr` are None.

Standalone test in `if __name__ == '__main__':`:
Load world.xml, step once, call `SensorReading.from_mujoco(model, data)`,
print all fields and assert types/shapes.

Done when: `python3 -m py_compile sensor_reading.py` passes and standalone test prints without error.

---

**Task 10.11 — `sim/task_tree/task_node.py`**

Dataclass `TaskNode` — one node in the task tree.

```python
from dataclasses import dataclass, field
from typing import Callable, List

@dataclass
class TaskNode:
    name:           str
    instruction:    str                               # language instruction fed to VLA
    primitives:     List[str]                         # named sub-actions (for logging)
    postcondition:  Callable[["SensorReading"], bool] # returns True when subtask is done
    timeout_steps:  int  = 3000                       # fail if not done in this many steps
    status:         str  = "pending"                  # pending | active | done | failed
    steps_active:   int  = field(default=0, repr=False)

    def tick(self, reading) -> str:
        """
        Call once per sim step when this node is active.
        Increments steps_active, evaluates postcondition, updates status.
        Returns new status string.
        """
```

The `tick()` method must:
- Increment `self.steps_active`
- If postcondition(reading) → set status = "done", return "done"
- Elif steps_active >= timeout_steps → set status = "failed", return "failed"
- Else → return "active"

Standalone test: create a TaskNode with a lambda postcondition, tick it to done and to timeout.

Done when: `python3 -m py_compile task_node.py` passes and standalone test passes both paths.

---

**Task 10.12 — `sim/task_tree/pick_and_place_tree.py`**

Function `build_pick_and_place_tree() -> list[TaskNode]` that returns the 7 nodes in order.

Import `TaskNode` and `SensorReading` from this package.
Import numpy as np. No MuJoCo imports.

Scene constants at top of file:
```python
TABLE_Z   = 0.85
TARGET_XY = np.array([0.5, 0.4])
```

Seven nodes with postcondition lambdas using ONLY fields from `SensorReading`:

```python
nodes = [
    TaskNode(
        name="SEARCH",
        instruction="locate the metal box on the table",
        primitives=["observe_overhead", "observe_wrist"],
        postcondition=lambda r: np.all(np.isfinite(r.box_pos)),
        timeout_steps=500,
    ),
    TaskNode(
        name="APPROACH",
        instruction="move above the metal box",
        primitives=["move_to_preapproach", "descend_to_box"],
        postcondition=lambda r: 0.0 < r.proximity < 0.025,
        timeout_steps=5000,
    ),
    TaskNode(
        name="GRASP",
        instruction="grasp the metal box",
        primitives=["activate_em"],
        postcondition=lambda r: r.em_active and np.linalg.norm(r.ee_force) > 4.0,
        timeout_steps=2000,
    ),
    TaskNode(
        name="LIFT",
        instruction="lift the metal box up",
        primitives=["lift_to_carry_height"],
        postcondition=lambda r: (
            np.linalg.norm(r.ee_force) > 4.0 and
            r.box_pos[2] > TABLE_Z + 0.15
        ),
        timeout_steps=3000,
    ),
    TaskNode(
        name="LOCATE_TARGET",
        instruction="find the red target zone",
        primitives=["observe_overhead"],
        postcondition=lambda r: np.all(np.isfinite(r.ee_pos)),
        timeout_steps=500,
    ),
    TaskNode(
        name="TRANSPORT",
        instruction="move the box to the red target zone",
        primitives=["move_to_above_target"],
        postcondition=lambda r: (
            np.linalg.norm(r.ee_pos[:2] - TARGET_XY) < 0.08 and
            r.box_pos[2] > TABLE_Z + 0.08
        ),
        timeout_steps=6000,
    ),
    TaskNode(
        name="PLACE",
        instruction="release the metal box onto the target zone",
        primitives=["deactivate_em"],
        postcondition=lambda r: r.target_contact > 0.0 and not r.em_active,
        timeout_steps=2000,
    ),
]
```

Standalone test: call `build_pick_and_place_tree()`, assert len == 7, print all node names and instructions.

Done when: `python3 -m py_compile pick_and_place_tree.py` passes and standalone test prints 7 nodes.

---

**Task 10.13 — `sim/task_tree/task_tree_manager.py`**

Class `TaskTreeManager` — the meta-controller.

```python
class TaskTreeManager:
    def __init__(self, nodes: list[TaskNode]):
        self._nodes = nodes
        self._idx   = 0
        self._activate_current()

    def step(self, reading: SensorReading) -> str:
        """
        Call once per sim step.
        Ticks the active node. Advances to next node on 'done'.
        Returns the language instruction for the current active node.
        """

    def get_current_instruction(self) -> str:
        """Return the VLA instruction for the current subtask."""

    def get_status_dict(self) -> dict:
        """
        Return JSON-serializable status for /task_tree/status topic.
        Format:
        {
            "active_node":   str,
            "active_idx":    int,
            "instruction":   str,
            "steps_active":  int,
            "node_statuses": [{"name": str, "status": str}, ...]
        }
        """

    def is_complete(self) -> bool:
        """True when the last node reaches 'done'."""

    def is_failed(self) -> bool:
        """True when any node reaches 'failed'."""

    def reset(self):
        """Reset all nodes to pending, restart from node 0."""
```

Standalone test: build the tree, create a manager, create a fake `SensorReading` that satisfies
each postcondition in turn, step through all 7 nodes, assert `is_complete() == True`.
The fake reading must satisfy postconditions 1 and 5 (SEARCH and LOCATE_TARGET) immediately.

Done when: `python3 -m py_compile task_tree_manager.py` passes and standalone test reaches `is_complete() == True`.

---

**Task 10.14 — `sim/task_tree/test_task_tree.py`**

Integration test: drive the full pick-and-place tree with a real MuJoCo simulation
using scripted primitives (not VLA).

Structure:
1. Load world.xml, set up PD gains.
2. Build task tree via `build_pick_and_place_tree()`.
3. Create `TaskTreeManager(nodes)`.
4. Main loop — at each step:
   a. Advance MuJoCo physics (or call the appropriate scripted primitive).
   b. Read `SensorReading.from_mujoco(model, data)`.
   c. Call `manager.step(reading)`.
   d. Print `manager.get_status_dict()` when subtask changes.
   e. Break on `is_complete()` or `is_failed()`.
5. Assert `is_complete() == True` at end.
6. Print final status dict.

Use the existing scripted demo logic (from `demo_full_sequence.py`) to drive each subtask.
Do not duplicate IK code — import `_ik_utils` and `em_controller` as before.

Output: `outputs/task_tree_test_<date>.mp4` (overhead + wrist side-by-side, same as demo_full_sequence).

Done when: test runs end-to-end and prints `is_complete() == True` without error.

---

### Group E — Octo real inference (deferred)

**Task 10.15 — Kaggle notebook**
Load Octo from HuggingFace. Accept observation via HTTP (ngrok).
Return 7-float action. Document the ngrok URL setup.

**Task 10.16 — Full VLA run**
Replace scripted primitives in test_task_tree.py with real Octo inference from Kaggle.
Record full pick-and-place attempt. Save video.
Document: how far did the model get? What failed?

---

## 8. SESSION_LOG format

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

## 9. What to do when you open this repo cold

1. Read this file end-to-end.
2. `git log --oneline -5` — see where we are.
3. Read the last entry in `VLATraining/sim/SESSION_LOG.md`.
4. Read `PHASES.md` — check Phase 10 status.
5. Apply the target zone fix first (§7 PREREQUISITE) if not already done.
6. Continue from the next incomplete task in §7 above.
7. Never start a task without reading its Done criteria first.

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-27*
