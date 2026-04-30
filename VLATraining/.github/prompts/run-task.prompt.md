---
description: "Execute a task from sim_instructions.md. Use when: starting any task like '0.6', '1.4', '3.7', etc. from the step-by-step instruction file."
name: "Run Task"
argument-hint: "Task ID, e.g. 0.6 or 1.4 or 3.7"
agent: "agent"
tools: ["read_file", "create_file", "run_in_terminal", "get_errors"]
---

You are implementing a task from the VLA simulation step-by-step instructions.

Task requested: **$TASK** (e.g. 0.6 or 1.4)

## Step 1 — Read the task spec

Read [sim_instructions.md](../../sim_instructions.md) and find the section for the requested task number.
Extract:
- The **What to do** description
- The **Done when** acceptance criteria
- Any **Notes** listed

## Step 2 — State the files you will create or modify

List every file you will touch. Confirm the list before writing any code.

Project root for all files: `/home/shahar/Desktop/phase4/VLATraining/sim/`
(The instructions say `VLAResearch/sim/` — map that to `VLATraining/sim/` throughout.)

**One file per tool call** — project hard rule.

## Step 3 — Implement

For each file:
1. Create or edit **one file at a time**.
2. Run `get_errors` after each file if it is a Python file.
3. Fix any errors before proceeding to the next file.

Follow all conventions in [AGENTS.md](../../AGENTS.md):
- Joint names: A1–A6 exactly
- EM activation: `model.eq_active[em_weld_id]` toggle
- Sensor lookup by name, never hard-coded index
- Camera render: `mujoco.Renderer` (not deprecated `mjr_render`)
- All outputs → `VLATraining/sim/outputs/`
- No `input()` or interactive prompts
- venv: `source /home/shahar/Desktop/phase4/phase4_env/bin/activate`

## Step 4 — Verify the "Done when" condition

Run the relevant script or command and confirm every item in the "Done when" list.

```bash
source /home/shahar/Desktop/phase4/phase4_env/bin/activate
cd /home/shahar/Desktop/phase4/VLATraining/sim
python <script_for_task>.py
```

Report PASS/FAIL for each "Done when" item.

## Step 5 — Summary

One paragraph: what was built, which "Done when" items passed, and what the next task is.
