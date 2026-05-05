---
name: start-of-day
description: "Start of day workflow for the VLA simulation project. Use when: starting a session, beginning work, what did we do last time, where did we leave off, resume, good morning, let's get started, what's next, what's the plan, catch me up, pull latest, load context."
argument-hint: "Optional: which phase or topic to focus on today"
---

# Start of Day — VLA Tutorial

Load context from the last session, sync from GitHub, and give a clear entry point so work can begin immediately.

## When to Use
- Opening a new session
- "What's next?", "where did we leave off?", "catch me up"
- After a break of any length
- Before beginning any new phase

---

## Procedure

### Step 1 — Pull latest from GitHub
From the repo root:

```bash
cd ~/Desktop/VLA_Tutorial
git pull origin main
git log --oneline -10
```

Show the last 5 commit messages so we know what is already committed vs. any local changes.

```bash
git status
git diff --stat HEAD
```

Flag any uncommitted local changes — if there are any, ask the human whether to commit them before starting.

---

### Step 2 — Read current state
Read these two files in full:
- `VLATraining/sim/SESSION_LOG.md` — find the **most recent session block** at the top
- `PHASES.md` — find the first phase that is **not** `✅ Done`

---

### Step 3 — Report the situation
Output a concise briefing in this format:

```
## Session Start — YYYY-MM-DD

### Last session (YYYY-MM-DD)
What was completed: <one paragraph>
What was left mid-way: <if anything>

### Current phase status
✅ Phases complete: 0–N
🔜 Next phase: Phase N+1 — <title>

### Entry point for today
1. <First concrete task> — <file to edit, what the function should do>
2. <Second task>
3. ...

### Environment checklist
- [ ] venv activated: source ~/Desktop/phase4/phase4_env/bin/activate
- [ ] Working directory: cd ~/Desktop/VLA_Tutorial/VLATraining/sim
- [ ] ROS2 sourced (if needed): source /opt/ros/humble/setup.bash
```

---

### Step 4 — Activate the environment
Run this so the terminal is ready:

```bash
source ~/Desktop/phase4/phase4_env/bin/activate
cd ~/Desktop/VLA_Tutorial/VLATraining/sim
python -c "import mujoco; print('MuJoCo', mujoco.__version__, '— ready')"
```

If the import fails, report the error and stop — do not proceed until the environment is working.

---

### Step 5 — Confirm and ask
Ask the human:
> "Ready to start. Do you want to begin with task [X] or is there something specific you want to tackle first?"

Do not start implementing anything until the human confirms the direction.

---

## Key constants (for reference in all phases)

```python
# Standard gains (all test scripts)
_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.)
}

# Scene geometry
TABLE_Z   = 0.85
BOX_HALF  = 0.05
GAP       = 0.015
EM_THRESH = 0.05  # m — proximity required to activate weld
```

## Key paths

| What | Path |
|------|------|
| Sim scripts | `VLATraining/sim/scene/` |
| World XML | `VLATraining/sim/scene/world.xml` |
| Session log | `VLATraining/sim/SESSION_LOG.md` |
| Phase tracker | `PHASES.md` |
| Full instructions | `docs/sim_instructions.md` |
| ROS2 package | `VLATraining/vla_ws/src/mujoco_bridge/` |
