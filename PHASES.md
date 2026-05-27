# VLA Simulation — Phase Tracker

**Project:** VLA Research Simulation | KUKA KR6 + MuJoCo + ROS2  
**Author:** Shahar Cohen | BGU Mechatronics MSc  
**Last updated:** 2026-05-27
**Repo:** cohenshahar/VLA_Tutorial
**Owner:** Shahar Cohen (shaharag@post.bgu.ac.il)

This file is read by Claude Code at the start of every session in this repo.
Read it end-to-end before writing any code.

---

## Phase Status

| Phase | Title | Status | Key output |
|---|---|---|---|
| 0 | Environment Setup | ✅ Done | venv, MuJoCo verified, ROS2 Humble active, vla_ws initialized |
| 1 | KUKA KR6 R900 URDF | ✅ Done | `assets/urdf/kr6r900sixx.urdf`, `assets/mjcf/kr6r900sixx.xml` |
| 2 | World Setup | ✅ Done | `scene/world.xml` — ground, table, arm mounted, physics params set |
| 3 | Moving the Arm (Joint by Joint) | ✅ Done | Joint control, limit enforcement, Jacobian IK, trajectory video |
| 4 | Electromagnetic End-Effector | ✅ Done | EM pad geometry + `em_contact_site` on A6 flange |
| 5 | Scene Objects (Box + Target Zone) | ✅ Done | Free-body box, friction, target zone, touch sensor |
| 6 | Electromagnetic Attachment System | ✅ Done | Weld constraint, proximity pre-condition, full EM cycle video |
| 7 | Sensor Suite | ✅ Done | Joint pos/vel/torque, F/T, proximity, `SensorLogger` CSV |
| 8 | Camera Setup | ✅ Done | `cam_overhead`, `cam_side`, `cam_wrist`, `CameraPublisher`, multicam video |
| 9 | ROS2 Bridge | ✅ Characterized | 10 topics live + measured: RTF 0.756 with cameras (above 0.70 floor), cameras at 6 Hz ✅, high-freq topics ~75 Hz (GIL-bound, Phase 10) |
| 10 | Task Tree Manager | ✅ Done | `TaskNode`, `TaskTreeManager`, 7-subtask pick-and-place tree, integration test |
| 11 | OpenVLA / Local Octo | 🔜 Next | Real local closed-loop VLA control loop via JAX model |

---

## Key file locations

| What | Path |
|---|---|
| Sim codebase | `VLATraining/sim/` |
| World XML | `VLATraining/sim/scene/world.xml` |
| Arm loader | `VLATraining/sim/arm/load_arm.py` |
| EM controller | `VLATraining/sim/scene/em_controller.py` |
| Sensor logger | `VLATraining/sim/scene/sensor_logger.py` |
| Camera utils | `VLATraining/sim/scene/camera_utils.py` |
| ROS2 bridge package | `VLATraining/vla_ws/src/mujoco_bridge/` |
| Task tree (Phase 10) | `VLATraining/sim/task_tree/` |
| Renders / videos | `VLATraining/sim/outputs/` |
| Full instructions | `docs/sim_instructions.md` |
| Work plan (milestones) | `docs/sim_work_plan.md` |
| Architecture diagram | `docs/sim_block_diagram.mermaid` |
