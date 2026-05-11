# VLA Simulation — Phase Tracker

**Project:** VLA Research Simulation | KUKA KR6 + MuJoCo + ROS2  
**Author:** Shahar Cohen | BGU Mechatronics MSc  
**Last updated:** 2026-05-11

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
| 10 | Task Tree Manager | 🔜 Next | `TaskNode`, `TaskTreeManager`, 3-subtask pick-and-place sequence |
| 11 | OpenVLA on Kaggle | 🔜 Next | 7-element action output from openvla/openvla-7b at ~1–3 Hz |

---

## Next actions (Phase 10 entry point)

1. Create `task_tree/task_node.py` — `TaskNode` dataclass (name, language_instruction, primitives, postcondition_fn, status)
2. Create `task_tree/pick_and_place_tree.py` — root node + 3 child subtasks with postcondition lambdas
3. Create `task_tree/task_tree_manager.py` — `TaskTreeManager` with `step()`, `get_current_instruction()`, `get_status_dict()`
4. Connect manager to `SensorLogger` via `step_from_logger(logger)`

See full task details in [docs/sim_instructions.md](docs/sim_instructions.md) — Phase 10 starts at Task 10.1.

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
