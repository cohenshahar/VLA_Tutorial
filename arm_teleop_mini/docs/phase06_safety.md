# Phase 06 — Joint limits + safety clamp

**Goal**: Joints stop at URDF limits with a single WARN log; they do not exceed configured min/max.

**Concepts introduced**:
- Loading joint limits from a YAML param file
- Per-joint clamping logic
- Warn-once pattern (avoid log spam)

**Deliverables**:
- `arm_teleop/config/kr6_params.yaml` — real KR6 limits filled in
- `arm_teleop/mujoco_sim_node.py` — `_clamp_to_limits` implemented

**Smoke test**:
- Command: drive joint 1 into its upper limit with `q`
- Expected: joint stops rotating; exactly one `[WARN]` line printed; no further spam

**Notes** (filled in as we go):
