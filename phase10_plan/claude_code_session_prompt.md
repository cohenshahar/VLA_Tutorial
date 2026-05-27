# Claude Code — Session Prompt for Phase 10 (Task Tree Manager)

**Copy-paste this as the first message to Claude Code when starting a session.**

---

## Prompt

```
Read CLAUDE.md in this repo root first: ~/Desktop/VLA_Tutorial/CLAUDE.md
Read it end-to-end before doing anything else.

Then read:
- git log --oneline -5
- VLATraining/sim/SESSION_LOG.md  (last entry only)
- PHASES.md

Then follow this sequence exactly:

STEP 1 — Target zone fix (if not yet done)
Check scene/includes/objects.xml.
If target_zone body pos is still "0 0 0.86", change it to "0.5 0.4 0.86".
Also update TARGET_XY to np.array([0.5, 0.4]) in:
  - scene/demo_place_near.py
  - scene/demo_place_far.py
  - scene/demo_full_sequence.py
Commit: "Phase 10, prereq: fix target zone to [0.5, 0.4]"

STEP 2 — Verify demo_place_near.py still passes
Run: PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_place_near.py
Done when: script prints target_contact == True within 1000 steps. If it fails, fix before continuing.

STEP 3 — Implement Task Tree Manager (tasks 10.10–10.14 in CLAUDE.md §7)
Work through tasks in order:
  10.10 → task_tree/sensor_reading.py
  10.11 → task_tree/task_node.py
  10.12 → task_tree/pick_and_place_tree.py
  10.13 → task_tree/task_tree_manager.py
  10.14 → task_tree/test_task_tree.py

Rules for this session:
- Commit after EACH completed task (not at the end)
- git push origin main after every commit
- python3 -m py_compile on every new file before committing
- Run the standalone test in __main__ for every new file before committing
- Do NOT start Group E (Octo / Kaggle) — that is deferred
- Append a SESSION_LOG entry before the final commit

If you finish all task tree tasks, stop and report what was completed.
Do not move to Group E without explicit instruction.
```

---

## Setup on Ubuntu (run once before starting Claude Code)

```bash
# 1 — pull the latest CLAUDE.md
cd ~/Desktop/VLA_Tutorial
git pull

# 2 — verify CLAUDE.md was updated (should show 2026-05-27)
head -3 CLAUDE.md

# 3 — activate the venv
source VLATraining/.venv/bin/activate

# 4 — start Claude Code
claude
```

---

## Notes

- `task_tree/` is currently empty (only `__init__.py`). All 4 new files go there.
- All task tree files import from each other within the package — use relative imports or
  insert `sys.path` as done in the demo scripts.
- The standalone test for `test_task_tree.py` (10.14) drives the full simulation.
  It imports from `scene/` and `arm/` — path bootstrap follows `demo_full_sequence.py` pattern.
- For tasks that need the ROS2 bridge (Group C tasks 10.7–10.9):
  open a second terminal, source ROS2, and run the bridge before Claude Code.
  Task tree tasks (Group D) do NOT need the bridge — pure MuJoCo standalone.

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-27*
