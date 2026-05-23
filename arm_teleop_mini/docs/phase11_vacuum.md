# Phase 11 — Vacuum gripper (black box)

**Goal**: `ee:=vacuum` launch works; pressing `v` toggles the vacuum on/off with a visual change in MuJoCo.

**Concepts introduced**:
- ROS2 service (`std_srvs/SetBool`) server and client pattern
- Launch file parameterization via `ee_type`
- MuJoCo material/color swap at runtime (visual-only, black box)

**Deliverables**:
- `arm_teleop/gripper_node.py` — vacuum branch implemented
- `arm_teleop/launch/teleop_vacuum.launch.py` — fully implemented
- `models/kr6_with_vacuum.xml` — real model (copied + extended from kr6.xml)

**Smoke test**:
- Command: launch vacuum stack; press `v`
- Expected: tube color changes in viewer; `ros2 service call /vacuum/set std_srvs/SetBool "{data: true}"` also works

**Notes** (filled in as we go):
