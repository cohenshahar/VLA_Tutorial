# VLA Simulation — KUKA KR6 + MuJoCo + ROS2

**Author:** Shahar Cohen | BGU Mechatronics MSc  
**Advisor:** Prof. Amir Shapira  
**Updated:** 2026-05-05

A full simulation stack for Vision-Language-Action (VLA) research. A KUKA KR6 R900 sixx arm picks up a metal box with an electromagnetic end-effector and places it on a target zone — all driven by a hierarchical task tree and observable through a 3-camera sensor suite over ROS2.

---

## What's in this repo

```
VLA_Tutorial/
├── PHASES.md                     ← phase-by-phase progress tracker (0–11)
├── setup_phase4.sh               ← one-shot environment setup script
├── docs/
│   ├── sim_instructions.md       ← full 131-task step-by-step build guide
│   ├── sim_work_plan.md          ← milestone plan (M1–M10)
│   └── sim_block_diagram.mermaid ← architecture diagram
├── logs/                         ← session summaries and day plans
└── VLATraining/
    ├── sim/                      ← simulation codebase
    │   ├── arm/                  ← arm loader + Jacobian IK
    │   ├── assets/               ← URDF + MJCF robot models + meshes
    │   ├── scene/                ← world XML, EM controller, sensors, cameras
    │   ├── bridge/               ← ROS2 bridge (Phase 9)
    │   └── task_tree/            ← task tree manager (Phase 10)
    └── vla_ws/
        └── src/
            ├── kuka_kr6_support/ ← KUKA ROS2 support package
            └── mujoco_bridge/    ← ROS2 bridge package (Phase 9 scaffold)
```

---

## Stack

| Layer | Technology |
|---|---|
| Physics simulation | MuJoCo 3.x |
| Robot arm | KUKA KR6 R900 sixx (6-DOF) |
| End-effector | Electromagnetic pad (weld constraint) |
| Robot middleware | ROS2 Humble |
| VLA model | OpenVLA (`openvla/openvla-7b`) on Kaggle GPU |
| Language | Python 3.10+ |

---

## Phase progress

| Phase | Title | Status |
|---|---|---|
| 0 | Environment setup | ✅ Done |
| 1 | KUKA KR6 URDF → MuJoCo | ✅ Done |
| 2 | World (table, ground, physics) | ✅ Done |
| 3 | Joint control + Jacobian IK | ✅ Done |
| 4 | EM end-effector geometry | ✅ Done |
| 5 | Box + target zone + touch sensor | ✅ Done |
| 6 | EM attach/detach cycle | ✅ Done |
| 7 | Sensor suite (joint, F/T, proximity) | ✅ Done |
| 8 | 3-camera setup + multicam video | ✅ Done |
| 9 | ROS2 bridge (topics + commands) | 🔜 Next |
| 10 | Task Tree Manager | 🔜 Next |
| 11 | OpenVLA inference on Kaggle | 🔜 Next |

See [PHASES.md](PHASES.md) for full details and next actions.

---

## Quick start

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Activate the Python venv
cd VLATraining/sim
source venv/bin/activate

# 3. Run the full scene viewer
python scene/demo_viewer.py

# 4. Run the EM pick-and-place demo
python scene/em_pickup_test.py
```

---

## Key files

| File | Purpose |
|---|---|
| `sim/scene/world.xml` | Main MuJoCo scene (arm + table + box + target) |
| `sim/arm/load_arm.py` | Arm loader + `ik_jacobian()` solver |
| `sim/scene/em_controller.py` | EM activate/deactivate with proximity check |
| `sim/scene/sensor_logger.py` | `SensorLogger` — logs all 27 sensor channels to CSV |
| `sim/scene/record_multicam.py` | Renders all 3 cameras to side-by-side video |
| `sim/bridge/__init__.py` | ROS2 bridge entry point (Phase 9) |
| `sim/task_tree/__init__.py` | Task tree entry point (Phase 10) |
| `docs/sim_instructions.md` | Complete step-by-step build instructions |

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026*
