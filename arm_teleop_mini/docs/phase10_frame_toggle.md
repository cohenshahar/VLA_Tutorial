# Phase 10 — Numpad teleop + frame toggle

**Goal**: Numpad drives EE in XYZ; `m` toggles between base frame and tool frame.

**Concepts introduced**:
- curses numpad key codes
- Tool-frame rotation: `v_base = R_base_tool @ v_tool`
- Propagating frame intent via `Twist.header.frame_id`

**Deliverables**:
- `arm_teleop/keyboard_teleop_node.py` — numpad handling + `_on_frame_toggle` implemented
- `arm_teleop/cartesian_controller_node.py` — tool-frame rotation branch implemented
- `arm_teleop/kinematics.py` — `quat_to_rotation_matrix` implemented

**Smoke test**:
- Command: run full stack; press numpad `8`
- Expected: EE moves +X in base frame; press `m`, press numpad `8` → EE moves along tool Z-axis

**Notes** (filled in as we go):
