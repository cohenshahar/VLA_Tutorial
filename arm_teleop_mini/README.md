# arm_teleop_mini

A self-contained learning project for teleoperation of a KUKA KR6 6-DOF arm inside
MuJoCo using ROS2 Humble. The goal is to build practical intuition for ROS2 nodes,
MuJoCo physics, and hand-coded Jacobian IK — one phase at a time, filled in with
GitHub Copilot.

---

## Phases

| # | Title |
|---|-------|
| 1 | Repo skeleton — buildable empty ROS2 package |
| 2 | MuJoCo bring-up — KR6 visible in viewer |
| 3 | `mujoco_sim_node` + `/joint_states` publisher |
| 4 | `/cmd_joint_vel` subscriber |
| 5 | Keyboard teleop — joint mode |
| 6 | Joint limits + safety clamp |
| 7 | FK + `/ee_pose` publisher |
| 8 | Jacobian IK math (notebook) |
| 9 | Cartesian controller — base frame |
| 10 | Numpad teleop + frame toggle |
| 11 | Vacuum gripper (black box) |
| 12 | Claw gripper (black box) |

---

## How to run

```bash
# Vacuum end-effector
ros2 launch arm_teleop teleop_vacuum.launch.py   # TODO Phase 11

# Claw end-effector
ros2 launch arm_teleop teleop_claw.launch.py     # TODO Phase 12

# View KR6 in MuJoCo standalone (Phase 2 smoke test)
python scripts/view_arm.py                        # TODO Phase 2
```
