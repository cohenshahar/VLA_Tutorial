# Phase 04 — /cmd_joint_vel subscriber

**Goal**: Simulator accepts joint velocity commands via `/cmd_joint_vel` and applies them to MuJoCo actuators.

**Concepts introduced**:
- `std_msgs/Float64MultiArray` subscriber
- Writing to `data.ctrl` in MuJoCo
- Command timestamping for deadman logic

**Deliverables**:
- `arm_teleop/mujoco_sim_node.py` — `_cmd_joint_vel_callback` implemented

**Smoke test**:
- Command: `ros2 topic pub /cmd_joint_vel std_msgs/Float64MultiArray "data: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]"`
- Expected: Joint 1 rotates continuously in the MuJoCo viewer

**Notes** (filled in as we go):
