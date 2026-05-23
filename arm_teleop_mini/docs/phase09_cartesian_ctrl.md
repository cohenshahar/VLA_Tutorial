# Phase 09 — Cartesian controller (base frame)

**Goal**: Publishing a `Twist` on `/cmd_ee_vel` drives the EE in the base frame via DLS IK.

**Concepts introduced**:
- `cartesian_controller_node` pipeline: Twist → DLS → joint velocities
- Subscribing to multiple topics and composing their data
- IK failure detection (condition number threshold)

**Deliverables**:
- `arm_teleop/cartesian_controller_node.py` — `_on_cmd_ee_vel` implemented for base frame

**Smoke test**:
- Command: `ros2 topic pub /cmd_ee_vel geometry_msgs/Twist "{linear: {x: 0.05}}"`
- Expected: EE moves in +X direction in the MuJoCo viewer; `/ee_pose` x-position increases

**Notes** (filled in as we go):
