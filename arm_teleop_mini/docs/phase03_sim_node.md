# Phase 03 — mujoco_sim_node + joint_states

**Goal**: Simulator ticks at 50 Hz and publishes `/joint_states` continuously.

**Concepts introduced**:
- `rclpy.node.Node` with a timer callback
- `sensor_msgs/JointState` message
- MuJoCo step loop inside a ROS2 node

**Deliverables**:
- `arm_teleop/mujoco_sim_node.py` — `__init__`, `_step_simulation`, `_publish_joint_states` implemented

**Smoke test**:
- Command: `ros2 topic hz /joint_states`
- Expected: `average rate: 50.000`

**Notes** (filled in as we go):
