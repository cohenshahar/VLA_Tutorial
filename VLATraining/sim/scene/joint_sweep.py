"""
Joint sweep demo — each of A1–A6 moves separately, one at a time.
All other joints are kinematically locked at 0° (home position).

Run from VLATraining/sim/:
    python -m scene.joint_sweep

Watch the MuJoCo viewer. Each joint sweeps to its target, holds,
returns to 0°, then the next joint takes over.
"""

import math, os, time
import mujoco
import mujoco.viewer

# ── Sweep definition ──────────────────────────────────────────────────────
# (joint_name, actuator_name, target_deg, KP, KV, forcerange_Nm)
# Target angles chosen to be clearly visible and within safe limits.
JOINTS = [
    ("joint_a1", "act_a1",  +60.0,  800.0, 80.0, 500.0),  # base rotation
    ("joint_a2", "act_a2",  -45.0,  800.0, 80.0, 500.0),  # shoulder down
    ("joint_a3", "act_a3",  +60.0,  500.0, 50.0, 300.0),  # elbow up
    ("joint_a4", "act_a4",  +90.0,  300.0, 30.0, 200.0),  # forearm roll
    ("joint_a5", "act_a5",  +45.0,  500.0, 50.0, 250.0),  # wrist tilt  (boosted: gravity on horiz arm)
    ("joint_a6", "act_a6",  +90.0,  300.0, 30.0, 250.0),  # wrist spin  (boosted: 90° needs ~157 Nm)
]
LABELS = [
    "A1 — base rotation",
    "A2 — shoulder",
    "A3 — elbow",
    "A4 — forearm roll",
    "A5 — wrist tilt",
    "A6 — wrist spin",
]

RAMP_T    = 1.0   # s  — ramp from 0° to target
HOLD_T    = 0.5   # s  — hold at target
PAUSE_T   = 0.3   # s  — pause at 0° before next joint
PHASE_DUR = 2 * RAMP_T + HOLD_T + PAUSE_T   # 2.8 s per joint

# ── Load model ─────────────────────────────────────────────────────────────
SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world_test.xml")

model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

# Pre-fetch IDs
jnt_info = [(model.joint(jn).qposadr[0], model.joint(jn).dofadr[0])
            for jn, *_ in JOINTS]
act_ids  = [model.actuator(an).id for _, an, *_ in JOINTS]


def activate_joint(idx):
    """Set KP/KV for the active joint; zero-out all others."""
    for i, (_, _, _, kp, kv, fr) in enumerate(JOINTS):
        ai = act_ids[i]
        if i == idx:
            model.actuator_gainprm[ai, 0] =  kp
            model.actuator_biasprm[ai, 1] = -kp   # must match gainprm[0]
            model.actuator_biasprm[ai, 2] = -kv
            model.actuator_forcerange[ai] = [-fr, fr]
        else:
            model.actuator_gainprm[ai, 0] = 0.0
            model.actuator_biasprm[ai, 1] = 0.0
            model.actuator_biasprm[ai, 2] = 0.0


# ── Initialise ─────────────────────────────────────────────────────────────
mujoco.mj_forward(model, data)
data.ctrl[:] = 0.0
activate_joint(0)

total_phases = len(JOINTS)
print("Joint sweep demo — A1 through A6, one at a time.")
print(f"Phase duration: {PHASE_DUR:.1f}s  |  Total: ~{total_phases * PHASE_DUR:.1f}s")
print("─" * 50)

# ── Viewer loop ─────────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0.3, 0.0, 1.2]
    viewer.cam.distance  = 2.8
    viewer.cam.azimuth   = 135
    viewer.cam.elevation = -20

    wall_start = time.time()
    phase_idx  = 0
    step       = 0
    last_print_phase_step = -1

    while viewer.is_running():
        # ── All done: hold at home ─────────────────────────────────────────
        if phase_idx >= total_phases:
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)
            continue

        t       = data.time
        phase_t = t - phase_idx * PHASE_DUR

        # ── Announce new phase ────────────────────────────────────────────
        if step == 0 or (phase_t < model.opt.timestep * 2 and
                         last_print_phase_step != phase_idx):
            last_print_phase_step = phase_idx
            _, _, tgt, *_ = JOINTS[phase_idx]
            print(f"\n→ {LABELS[phase_idx]}  (target {tgt:+.0f}°)")

        # ── Kinematic lock: force all non-active joints to 0° ─────────────
        for i, (qpa, doa) in enumerate(jnt_info):
            if i != phase_idx:
                data.qpos[qpa] = 0.0
                data.qvel[doa] = 0.0

        # ── Compute setpoint for active joint ─────────────────────────────
        _, _, target_deg, *_ = JOINTS[phase_idx]
        target_rad = math.radians(target_deg)

        if phase_t < RAMP_T:                          # ramp to target
            setpoint = (phase_t / RAMP_T) * target_rad
        elif phase_t < RAMP_T + HOLD_T:              # hold at target
            setpoint = target_rad
        elif phase_t < 2 * RAMP_T + HOLD_T:          # ramp back to 0
            frac = (phase_t - RAMP_T - HOLD_T) / RAMP_T
            setpoint = (1.0 - frac) * target_rad
        else:                                          # pause at 0
            setpoint = 0.0

        data.ctrl[act_ids[phase_idx]] = setpoint
        mujoco.mj_step(model, data)

        # ── Console progress every 0.5 s ──────────────────────────────────
        if step % 500 == 0:
            qpa, _ = jnt_info[phase_idx]
            theta  = math.degrees(data.qpos[qpa])
            print(f"  t={t:5.2f}s  phase_t={phase_t:.2f}s  "
                  f"θ={theta:+6.1f}°  (setpoint {math.degrees(setpoint):+.1f}°)")

        # ── Real-time throttle ────────────────────────────────────────────
        wall_target = wall_start + t
        lag = wall_target - time.time()
        if lag > 0:
            time.sleep(lag)

        viewer.sync()
        step += 1

        # ── Advance to next phase ─────────────────────────────────────────
        if phase_t >= PHASE_DUR:
            data.ctrl[act_ids[phase_idx]] = 0.0
            phase_idx += 1
            if phase_idx < total_phases:
                activate_joint(phase_idx)
            else:
                print("\n── All joints swept. Close the viewer to exit. ──")

print("\nDone.")
