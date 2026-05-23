# Phase 07 — FK + /ee_pose publisher

**Goal**: End-effector pose is published in the base frame at 50 Hz.

**Concepts introduced**:
- Reading body position and quaternion from `mujoco.MjData`
- MuJoCo quaternion convention: `[w, x, y, z]`
- `geometry_msgs/PoseStamped`

**Deliverables**:
- `arm_teleop/mujoco_sim_node.py` — `_publish_ee_pose` implemented

**Smoke test**:
- Command: `ros2 topic echo /ee_pose`
- Expected: pose values update as joints move; position is plausible for KR6 reach (~0.5–1.5 m from base)

**Notes** (filled in as we go):
