# Phase 02 — MuJoCo + KR6 bring-up

**Goal**: KR6 is visible in the MuJoCo interactive viewer via a standalone Python script.

**Concepts introduced**:
- MuJoCo Python bindings (`mujoco.MjModel`, `mujoco.MjData`)
- MJCF model loading
- `mujoco.viewer.launch`

**Deliverables**:
- `models/kr6.xml` (copied from Phase 9 KR6 setup, replacing the placeholder)
- `scripts/view_arm.py` (implementation filled in)

**Smoke test**:
- Command: `python scripts/view_arm.py`
- Expected: MuJoCo viewer window opens showing the KR6 arm model

**Notes** (filled in as we go):
