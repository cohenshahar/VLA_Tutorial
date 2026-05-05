# Simulation Environment — Work Plan (v3)
**Date:** 2026-04-28 | **Phase:** 1 — Simulation Foundation
**Author:** Shahar Cohen | BGU Mechatronics MSc
**Advisor:** Dr. Barak Or
**Related diagram:** `sim_block_diagram.mermaid`
**Revision note v3:** Arm confirmed from video — KUKA KR AGILUS KR 6 R900 sixx. Bare flange (no EE currently). No cameras on arm or environment. Workspace: Al-extrusion table, safety cage, flat metal surface ~600–700 mm deep.

---

## Execution environment

All milestones run in a **Kaggle or Google Colab notebook with GPU** (T4 minimum; A100 preferred for OpenVLA inference).
MuJoCo runs headlessly (`pip install mujoco`). ROS2 Humble is installed via shell cells in the notebook, or replaced by a pure-Python message bus in early milestones (see M4).

> **Rule:** No code until Shahar approves diagram + this plan.
> **Rule:** One file per tool call.
> **Rule:** No human in the loop — every state transition is autonomous.

---

## Confirmed decisions

| Decision | Choice |
|---|---|
| ROS version | ROS2 Humble (with pure-Python bus fallback for Colab) |
| Robot arm | **KUKA KR AGILUS KR 6 R900 sixx** (confirmed from video) |
| URDF source | `ros-industrial/kuka_experimental` → `kuka_kr6_support` package |
| End-effector | Electromagnetic — weld-constraint approach in MuJoCo |
| Task domain | Pick-and-place of a metal box using electromagnetic attachment |
| Task tree depth | 3 levels: Task → Subtask → Primitive actions |
| VLA inference | OpenVLA local (GPU notebook) |
| Compute | Kaggle / Google Colab GPU |
| Simulation fidelity | Full realistic physics, real sensor models, multi-camera |
| Camera placement | Wrist-mounted (end-effector) + ≥2 fixed environment cameras |

---

## Architecture overview — sensor & actuator map

```
KUKA 6-DOF arm (MuJoCo MJCF)
│
├── Joints (6×)
│     ├── Sensors: joint position encoder (qpos), velocity (qvel), torque (actuator_force)
│     └── Actuators: position + torque control via data.ctrl
│
├── End-effector link
│     ├── Electromagnetic actuator
│     │     └── Active/inactive flag → MuJoCo weld equality constraint
│     ├── Force/torque sensor (6-axis) — mujoco sensor type "force" + "torque"
│     ├── Proximity sensor (rangefinder) — triggers EM pre-condition check
│     └── Wrist camera (RGB, 640×480, 6 Hz) — moves with end-effector
│
└── Environment (fixed bodies)
      ├── Table (rigid, friction-configured)
      ├── Metal box (free body, ferromagnetic tag, mass + inertia realistic)
      ├── Target zone (visual marker + contact sensor)
      ├── Environment camera 1: overhead bird's-eye (RGB, 640×480, 6 Hz)
      └── Environment camera 2: angled side/front view (RGB, 640×480, 6 Hz)
```

---

## Milestone plan

---

### M1 — KUKA KR 6 R900 sixx URDF → MuJoCo MJCF ✅ COMPLETE (2026-04-29)
**Goal:** Load the confirmed lab arm (KUKA KR AGILUS KR 6 R900 sixx) into MuJoCo and verify geometry, joint count, and coordinate frames.

**Arm specs (KR 6 R900 sixx):**

| Parameter | Value |
|---|---|
| Payload | 6 kg |
| Reach | 901 mm |
| Joints | 6 revolute (A1–A6) |
| Repeatability | ±0.03 mm |
| Weight | 54 kg |
| Controller | KR C4 |

Steps:
- Clone `ros-industrial/kuka_experimental`: `git clone https://github.com/ros-industrial/kuka_experimental`
- Navigate to `kuka_kr6_support/urdf/` → `kr6r900sixx.xacro` + mesh files
- Convert to MJCF: use `python -m mujoco.robot_parser` or the `mjcf` Python API with `mjcf.from_path(urdf_path)` after resolving xacro
- Add an EM end-effector mount body at the A6 flange (bare in real arm — confirmed from video)
- Verify: `model.nq == 6`, joint names A1–A6, joint limits match KR 6 R900 datasheet, visual meshes load
- Render offscreen frame and save PNG

**Done when:** PNG shows KUKA arm in home configuration (all joints at 0°). `model.nq == 6`. Joint limits verified: A1 ±170°, A2 −190°/+45°, A3 −120°/+156°, A4 ±185°, A5 ±120°, A6 ±350°.

**✅ Verified (2026-04-29):** `model.nq == 6`. All joints load. Critical fix: `<compiler angle="radian"/>` required — without it MuJoCo treats radian range values as degrees, capping every joint at ~3°. Arm visualised in live viewer.

**Workspace notes from video:**
- Table surface: flat metal, aluminum extrusion frame, ~600–700 mm depth
- Safety cage around arm — model the cage walls as static collision geometry
- Second device visible in background left — add as static obstacle body
- No cameras currently on arm or environment — all cameras are new additions

---

### M2 — Joint control with realistic physics ✅ COMPLETE (2026-04-29)
**Goal:** Send joint position commands and verify realistic dynamics — inertia, joint limits, friction, gravity compensation.

Steps:
- Configure MuJoCo actuators: `position` or `velocity` type with proper `kp`/`kv` gains matching KUKA specs
- Set joint limits, armature (motor inertia), damping, and friction per the KUKA datasheet
- Script a move sequence: home → reach pose → home; record `qpos`, `qvel`, `actuator_force` per step
- Verify: no joint limit violations, no unphysical joint snapping, gravity visually correct

**Done when:** Arm moves from home to target pose and back. `actuator_force` log shows realistic torque values (e.g., shoulder joint ≈ 80–150 Nm for KR6). Gravity compensation visible in torque data when arm is extended.

**✅ Verified (2026-04-29):**
- `scene/tune_j1.py`: A1 sweeps 0°→60°, settles in 1.12s, 0.7% overshoot. KP=800, KV=80, DAMPING=10, ARMATURE=0.5.
- `scene/joint_sweep.py`: A1–A6 swept individually in live viewer. A1–A4 clean. A5/A6 gains refined.
- Confirmed: `biasprm[1]` must equal `-gainprm[0]` at runtime or equilibrium shifts.
- Confirmed: spring-locking fails under gravity; kinematic locking (force qpos+qvel each step) is reliable.

---

### M3 — Full pick-and-place scene with realistic physics
**Goal:** Build the complete environment with accurate contact physics, friction, and object dynamics.

Scene elements:
- Table: rigid body, friction `(1.0, 0.005, 0.0001)` (sliding, torsional, rolling) — MuJoCo defaults realistic
- Metal box: free body (`freejoint`), realistic mass (e.g. 0.5 kg), inertia tensor matching a 10×10×10 cm steel cube
- Target zone: visual geom (no physics) + a `site` for distance measurement
- End-effector mount: rigid attachment point for the EM pad

Electromagnetic end-effector (MuJoCo implementation):
- Add a `weld` equality constraint between the end-effector site and the box body — initially `inactive`
- `EM_activate()`: check proximity via rangefinder sensor; if distance < 2 cm AND magnet ON → set `model.eq_active[em_weld_id] = True`
- `EM_deactivate()`: set `model.eq_active[em_weld_id] = False` → box released
- This produces physically realistic attach/detach with no constraint violation artifacts

**Done when:** Arm can lift the box via EM activation (box follows end-effector). Box drops immediately on EM deactivation. Contact forces are non-zero during holding. Box does not fall through table.

---

### M4 — Sensor suite
**Goal:** Instrument the simulation with all sensors the real KUKA arm would have. All sensors publish on the bus.

Sensors to implement:

| Sensor | MuJoCo type | Publish topic | Rate |
|---|---|---|---|
| Joint encoders (6×) | `jointpos`, `jointvel` | `/joint_states` | 1000 Hz (sim), 100 Hz (bus) |
| Joint torques (6×) | `actuatorfce` | `/joint_torques` | 100 Hz |
| End-effector F/T (6-axis) | `force` + `torque` on EE site | `/ft_sensor` | 100 Hz |
| Proximity / rangefinder | `rangefinder` on EE site | `/proximity` | 50 Hz |
| EM state | Custom flag | `/em_state` | on-change |
| Contact detector (target zone) | `touch` geom on target site | `/target_contact` | 50 Hz |

Steps:
- Add `<sensor>` blocks to MJCF for each sensor type
- Write `SensorPublisher` class: reads `data.sensordata` array (indexed per sensor), publishes to bus
- Verify: rangefinder reads ≈0 when EE touches box, ≈∞ when arm is in air; F/T reads non-zero during EM hold

**Done when:** All 6 sensor topics publish at correct rates. Rangefinder correctly reports distance to box during approach. F/T sensor shows load during lift that matches box weight (0.5 kg × 9.81 ≈ 4.9 N).

---

### M5 — Multi-camera perception pipeline
**Goal:** Stream RGB from all 3 cameras simultaneously into the message bus at 6 Hz.

Camera configuration:

| Camera | Attachment | Position (approximate) | Purpose |
|---|---|---|---|
| Wrist cam | End-effector body (moves with arm) | Facing forward from EE | VLA visual input; close-up of grasp |
| Env cam 1 (overhead) | Fixed world body | 1.5 m above table, pointing down | Full workspace view; postcondition verification |
| Env cam 2 (side) | Fixed world body | 1.2 m lateral, 45° angle | Depth disambiguation; object pose estimation |

Steps:
- Define all 3 cameras in MJCF using `<camera>` elements with correct `pos` and `quat`
- Write `CameraPublisher`: for each camera, call `mj.mjr_render` + `mj.mjr_readPixels` → numpy `uint8[H, W, 3]`
- Publish each stream to its own topic: `/camera/wrist/image_raw`, `/camera/overhead/image_raw`, `/camera/side/image_raw`
- Benchmark: all 3 cameras must render at ≥6 Hz combined on Colab GPU

**Done when:** 3 PNG samples (one per camera) are saved and visually correct. Combined render rate ≥6 Hz. Wrist camera image shows the box correctly during approach.

---

### M6 — ROS2 bridge (or Python bus)
**Goal:** Connect all sensors and actuators to a message-passing layer so every component is fully decoupled.

Full topic list:

```
/joint_states            sensor_msgs/JointState       100 Hz
/joint_torques           sensor_msgs/JointState       100 Hz
/joint_commands          trajectory_msgs/JointTrajectory  on-demand
/ft_sensor               geometry_msgs/WrenchStamped  100 Hz
/proximity               sensor_msgs/Range            50 Hz
/em_state                std_msgs/Bool                on-change
/target_contact          std_msgs/Bool                50 Hz
/camera/wrist/image_raw  sensor_msgs/Image            6 Hz
/camera/overhead/image_raw  sensor_msgs/Image         6 Hz
/camera/side/image_raw   sensor_msgs/Image            6 Hz
/task_tree/status        std_msgs/String (JSON)       on-change
```

Two options — test Option A first in Colab; fall back to B if install fails:

**Option A — ROS2 Humble in Colab/Kaggle:**
- Install via shell: `apt-get install ros-humble-desktop` + `source /opt/ros/humble/setup.bash`
- Write `MujocoRos2Bridge` node using `rclpy`

**Option B — Pure-Python SimBus (fallback, zero dependencies):**
- `SimBus`: thread-safe `dict[str, queue.Queue]` with `put(topic, msg)` / `get(topic)` / `subscribe(topic, callback)`
- Same interface — swap to ROS2 later with adapter only

**Done when:** All topics listed above are active. `ros2 topic hz /joint_states` (or bus equivalent) shows ≥100 Hz. All camera topics publish at ≥6 Hz.

---

### M7 — Task Tree Manager
**Goal:** Implement the 3-level hierarchical task tree for EM pick-and-place. This is the thesis core component.

Tree structure:
```
Task: EM Pick-and-place
├── Subtask 1: Approach
│     Primitives: [move_to_preapproach, descend_to_box]
│     Postcondition: proximity_sensor < 2 cm AND overhead_cam confirms box below EE
├── Subtask 2: Attach & Lift
│     Primitives: [activate_EM, verify_attach, lift_to_carry_height]
│     Postcondition: em_state == ON AND ft_sensor.Fz > 4.5 N AND box_z > table_z + 10 cm
└── Subtask 3: Transport & Release
      Primitives: [move_to_target_zone, descend_to_place_height, deactivate_EM]
      Postcondition: target_contact == True AND em_state == OFF AND ft_sensor.Fz ≈ 0
```

Steps:
- Implement `TaskNode` dataclass: `(name, language_instruction, primitives: list[str], postcondition_fn: Callable[SensorReading → bool])`
- Implement `TaskTreeManager`: current node tracker, advances on postcondition pass, emits language instruction to VLA
- Implement all 3 postcondition functions using live sensor bus data (proximity, F/T, EM state, overhead camera)
- Publish `/task_tree/status` as JSON: `{subtask_id, subtask_name, status, postcondition_result}`

**Done when:** `TaskTreeManager` correctly sequences all 3 subtasks when driven by a scripted policy (not yet OpenVLA). All postconditions evaluate correctly with live sensor data from the simulation.

---

### M8 — OpenVLA integration
**Goal:** Load OpenVLA in the GPU notebook and generate action tokens from image + language instruction.

Steps:
- `pip install transformers accelerate` in Colab; load `openvla/openvla-7b` from HuggingFace
- Write `OpenVLANode`: takes `(image: np.ndarray, instruction: str)` → `delta_joints: np.ndarray[7]`
- Input image: wrist camera frame (primary) — OpenVLA was trained on wrist-view data
- Normalize action tokens: use BridgeData v2 stats, or dataset-specific stats if KUKA data is available
- Benchmark: ≥1 Hz inference on T4 (target 6 Hz on A100)
- Note: OpenVLA outputs 7 values (6 joint + 1 gripper). Map gripper dim → EM activate/deactivate threshold

**Done when:** OpenVLA outputs a 7-element action array for a single (image, instruction) pair in ≤2 seconds. The 7th value correctly maps to EM on/off when thresholded at 0.

---

### M9 — Full closed-loop execution
**Goal:** Task Tree Manager drives OpenVLA drives KUKA via the bus. Full EM pick-and-place attempt, no human input.

Full data flow:
```
TaskTreeManager
  → language instruction (str)
  → OpenVLANode (wrist image + instruction)
    → 7-DoF Δ joints + EM command
    → /joint_commands + /em_command
    → MuJoCoSim (via bridge/bus)
      → physics step
      → SensorPublisher → all sensor topics
        → TaskTreeManager postcondition check
          → advance subtask OR hold
```

Steps:
- Wire all nodes in a single orchestrator `main.py`
- Run ≥3 full pick-and-place attempts
- Log all signals: joint states, torques, F/T, proximity, EM state, action tokens, subtask id, timestamps

**Done when:** System runs a full task attempt without crashing. Task tree sequences at least 2/3 subtasks correctly. F/T log shows realistic load during carry. All topics logged to CSV.

---

### M10 — Baseline logging & reproducibility
**Goal:** Deterministic, structured logging for all components. Replay any run offline.

Steps:
- Set seeds: `numpy`, `torch`, MuJoCo `model.opt.timestep`
- `Logger` class: one CSV per run, per component: `{run_id, t_sim, t_wall, qpos[6], qvel[6], action[7], ft[6], proximity, em_state, subtask_id, postcondition_result}`
- `replay.py`: load CSV → reconstruct joint trajectory → render Matplotlib animation
- Determinism test: two runs with same seed produce bit-identical logs

**Done when:** Determinism test passes. Matplotlib animation correctly reconstructs any saved run from logs alone.

---

## Open questions remaining

1. **KUKA model designation** — ✅ **Confirmed from video: KUKA KR AGILUS KR 6 R900 sixx.** Verify the exact label on the base plate to rule out KR 6 R700 variant — kinematics differ slightly.
2. **EM payload capacity** — What is the max box mass the lab EM gripper can hold? Sets the F/T sensor threshold in M7.
3. **OpenVLA action space** — Will we use BridgeData v2 normalization or fine-tune on KUKA data? Fine-tuning needed if the arm kinematics differ significantly.
4. **Camera mounting** — Approximate KUKA wrist mount position for the camera? (Needed for MJCF camera body attachment.)
5. **ROS2 in Colab** — Test feasibility in M6 before committing. If install is fragile, Option B (SimBus) is default.

---

## What is NOT in scope for these milestones

Phase 2 additions — do not build yet:

- TimesFM kinematic failure detector
- Semantic Verifier (VLM postcondition verification)
- Emergency Planner (hierarchical re-anchoring)
- Failure injection harness
- Evaluation metrics (Prediction Lead Time, FPR, Recovery Rate, Loop Count)

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-04-28 v3*
