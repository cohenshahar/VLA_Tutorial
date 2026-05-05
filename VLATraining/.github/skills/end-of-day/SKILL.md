---
name: end-of-day
description: "End of day workflow for the VLA simulation project. Use when: finishing a session, wrapping up, end of day, closing out, committing work, updating github, summarizing progress, writing session log, planning next session, what did we do today, push to github."
argument-hint: "Optional: extra notes to include in the summary"
---

# End of Day — VLA Tutorial

Wrap up the current session: summarize what was built, record it in SESSION_LOG.md, plan the next session, then commit and push to GitHub.

## When to Use
- End of a working session
- "Let's wrap up", "end of day", "commit everything", "update github"
- After completing a phase or a set of tasks

---

## Procedure

### Step 1 — Read current state
Read these files to understand what changed today:
- `VLATraining/sim/SESSION_LOG.md` — ground truth of all phases
- `PHASES.md` — phase completion tracker
- Git diff to see exactly what changed: `git diff HEAD` and `git status`

### Step 2 — Build the session summary
Write a new session block at the **top** of `SESSION_LOG.md` (below the file header) with this format:

```
## Session YYYY-MM-DD — Phase X: <Title>

### What changed
| File | Type | Summary |
|------|------|---------|
| ... | NEW / Updated / Fixed | one-line description |

### Design decisions
Any non-obvious choices made today and why.

### Test results
Paste key pass/fail lines from any tests run.

### Phase X task completion status
| Task | File | Status |
|------|------|--------|
| X.Y — description | file | ✅ / 🔄 / ❌ |
```

### Step 3 — Update PHASES.md
- Mark any newly completed phases as `✅ Done`
- Update the "Next actions" section with the concrete first steps for the next session
- Keep the key file locations table accurate

### Step 4 — Plan next session
At the bottom of the new SESSION_LOG entry, add:

```
### Next session — entry point
1. <First concrete task> — file to edit, what to do
2. <Second task>
3. ...
```

Be specific: file names, function names, what the test should print when done.

### Step 5 — Stage and commit
Run these commands (from the repo root `~/Desktop/VLA_Tutorial`):

```bash
cd ~/Desktop/VLA_Tutorial
git add -A
git status   # review what will be committed
git commit -m "Session YYYY-MM-DD: <one-line summary of what was done>"
```

Commit message format: `Session YYYY-MM-DD: Phase X complete — <key achievement>`

Examples:
- `Session 2026-05-05: Phase 8 cameras fixed + camera_utils.py`
- `Session 2026-05-03: Phase 6 EM weld controller complete`

### Step 6 — Push to GitHub
```bash
git push origin main
```

Confirm push succeeded (exit code 0, no errors).

### Step 7 — Report to user
Print a clean end-of-day summary:

```
═══════════════════════════════════════════
  END OF DAY — YYYY-MM-DD
═══════════════════════════════════════════
  Phase:     X — <Title>
  Committed: <N> files
  Pushed:    ✓ origin/main

  Completed today:
    ✅ Task X.Y — description
    ✅ Task X.Z — description

  Next session starts at:
    → Task Y.1 — <what to do> in <file>

  Branch: main
  Remote: https://github.com/cohenshahar/VLA_Tutorial
═══════════════════════════════════════════
```

---

## Key file locations
| File | Purpose |
|------|---------|
| `VLATraining/sim/SESSION_LOG.md` | Ground truth — all phase history |
| `PHASES.md` | Phase status tracker |
| `VLATraining/sim/scene/world.xml` | Main scene XML |
| `VLATraining/sim/arm/load_arm.py` | Shared arm utilities |
| `VLATraining/sim/scene/em_controller.py` | EM weld controller |
| `VLATraining/sim/scene/sensor_logger.py` | Sensor CSV logger |
| `VLATraining/sim/scene/camera_utils.py` | Camera rendering utility |

## Rules
- Never force-push (`--force`)
- Never amend a published commit
- If push fails (auth / conflict), report the error to the user and stop — do not retry destructively
- Do not delete any output files (`.png`, `.mp4`, `.csv`) — they are project artifacts
- The commit must include SESSION_LOG.md and PHASES.md updates
