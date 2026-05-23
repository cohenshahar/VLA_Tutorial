# Phase 12 — Claw gripper (black box)

**Goal**: `ee:=claw` launch works; `[` opens and `]` closes the fingers with animation in MuJoCo.

**Concepts introduced**:
- Two hinge joints as finger actuators in MJCF
- Driving finger `qpos` targets from a service callback
- Launch file swap: model path + `ee_type` param

**Deliverables**:
- `arm_teleop/gripper_node.py` — claw branch implemented
- `arm_teleop/launch/teleop_claw.launch.py` — fully implemented
- `models/kr6_with_claw.xml` — real model (copied + extended from kr6.xml)

**Smoke test**:
- Command: launch claw stack; press `[` then `]`
- Expected: fingers animate open/close in viewer; `ros2 service call /claw/set` also works

**Notes** (filled in as we go):
