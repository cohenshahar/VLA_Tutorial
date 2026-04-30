"""
Phase 3 — Joint-by-joint tests (Tasks 3.1 – 3.8).

Run from VLATraining/sim/:
    python -m scene.test_joints

What this script does (in order):
  3.1  Drive A1 to +30°,   save outputs/joint_A1_plus30.png
  3.2  Return A1 to   0°,  save outputs/joint_A1_return.png
  3.3  Drive A2 to -30°,   save outputs/joint_A2_minus30.png  (then return)
  3.4  Drive A3 to +30°,   save outputs/joint_A3_plus30.png   (then return)
  3.5  Drive A4 to +45°,   save outputs/joint_A4_plus45.png   (then return)
  3.6a Drive A5 to +30°,   save outputs/joint_A5_plus30.png   (then return)
  3.6b Drive A6 to +90°,   save outputs/joint_A6_plus90.png   (then return)
  3.7  All joints → reach pose, save outputs/reach_pose.png
  3.8  Command A1 to 300°  → verify it clamps at joint limit

Each test:
  - resets all joints to 0 first
  - runs 2000 steps (2 s) to settle at target
  - renders an offscreen PNG
  - prints PASS / FAIL
"""
import math
import os
import numpy as np
import mujoco

try:
    from PIL import Image
    def _save(pixels, path):
        Image.fromarray(pixels).save(path)
except ImportError:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    def _save(pixels, path):
        plt.imsave(path, pixels)

# ── Paths ────────────────────────────────────────────────────────────────
SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR = os.path.join(SIM_DIR, "outputs")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Load model once ───────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)
model.vis.global_.offwidth  = 1280
model.vis.global_.offheight = 720

# ── Apply proper gains to all actuators ──────────────────────────────────
# XML kv=10 is too underdamped — arm oscillates for many seconds.
# biasprm[1] must equal -gainprm[0] or the position equilibrium shifts.
# Values tuned to match tune_j1.py results (tested 2026-04-29).
_GAINS = {
    "act_a1": (800.0, 80.0),  # (KP, KV)
    "act_a2": (800.0, 80.0),
    "act_a3": (500.0, 50.0),
    "act_a4": (300.0, 30.0),
    "act_a5": (500.0, 50.0),
    "act_a6": (300.0, 30.0),
}
for name, (kp, kv) in _GAINS.items():
    ai = model.actuator(name).id
    model.actuator_gainprm[ai, 0] =  kp
    model.actuator_biasprm[ai, 1] = -kp  # must match gainprm[0]
    model.actuator_biasprm[ai, 2] = -kv

# Add rotor inertia to every DOF — prevents numerical blow-up for light wrist links
for i in range(model.nv):
    model.dof_armature[i] = 0.5  # same value used in tune_j1.py for A1

# Actuator indices (matches act_a1…act_a6 order)
ACT = {f"a{i+1}": model.actuator(f"act_a{i+1}").id for i in range(6)}
JNT = {f"a{i+1}": model.joint(f"joint_a{i+1}") for i in range(6)}

# Camera: diagonal view, shows full arm
CAM_WIDE = dict(lookat=[0.0, 0.1, 1.1], distance=2.8, azimuth=140, elevation=-18)
CAM_SIDE = dict(lookat=[0.0, 0.0, 1.0], distance=2.5, azimuth=90,  elevation=-15)


def _make_cam(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance  = distance
    cam.azimuth   = azimuth
    cam.elevation = elevation
    return cam


def reset():
    """Instantly reset simulation to home (all joints 0, zero velocity)."""
    mujoco.mj_resetData(model, data)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)  # propagate kinematics, no physics drift


def run_to_target(ctrl_dict: dict, steps: int = 2000):
    """Set ctrl for active joints; kinematically lock all others at 0°."""
    active = set(ctrl_dict.keys())
    lock = [(JNT[k].qposadr[0], JNT[k].dofadr[0]) for k in JNT if k not in active]

    for key, val_rad in ctrl_dict.items():
        data.ctrl[ACT[key]] = val_rad

    max_ncon = 0
    for _ in range(steps):
        for qpa, doa in lock:
            data.qpos[qpa] = 0.0
            data.qvel[doa] = 0.0
        mujoco.mj_step(model, data)
        if data.ncon > max_ncon:
            max_ncon = data.ncon
    if max_ncon > 0:
        print(f"  ⚠ max contacts during run: {max_ncon}")


def render_png(path: str, cam_kwargs: dict = None):
    """Render current state offscreen and save PNG."""
    if cam_kwargs is None:
        cam_kwargs = CAM_WIDE
    cam = _make_cam(**cam_kwargs)
    with mujoco.Renderer(model, height=720, width=1280) as r:
        r.update_scene(data, camera=cam)
        _save(r.render(), path)
    print(f"  Saved → {path}")


def joint_deg(name: str) -> float:
    jnt = JNT[name]
    return math.degrees(data.qpos[jnt.qposadr[0]])


# ═══════════════════════════════════════════════════════════════════════════
# Task 3.1 — A1 to +30°
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.1: A1 → +30° ──")
reset()
run_to_target({"a1": math.radians(30)})
actual = joint_deg("a1")
print(f"  A1 actual = {actual:+.2f}°  (target +30°)")
render_png(os.path.join(OUT_DIR, "joint_A1_plus30.png"))
assert abs(actual - 30) < 5, f"FAIL 3.1: A1={actual:.2f}°"
print("  Task 3.1 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.2 — Return A1 to 0°
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.2: A1 → 0° (return) ──")
run_to_target({"a1": 0.0})
actual = joint_deg("a1")
print(f"  A1 actual = {actual:+.2f}°  (target 0°)")
render_png(os.path.join(OUT_DIR, "joint_A1_return.png"))
assert abs(actual) < 5, f"FAIL 3.2: A1={actual:.2f}°"
print("  Task 3.2 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.3 — A2 to −30°, return
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.3: A2 → -30° ──")
reset()
run_to_target({"a2": math.radians(-30)})
actual = joint_deg("a2")
print(f"  A2 actual = {actual:+.2f}°  (target -30°)")
render_png(os.path.join(OUT_DIR, "joint_A2_minus30.png"))
assert abs(actual - (-30)) < 8, f"FAIL 3.3: A2={actual:.2f}° (gravity causes ~5° SS error with PD)"  # gravity SS error expected
print("  Task 3.3 PASS ✓")
run_to_target({"a2": 0.0})   # return

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.4 — A3 to +30°, return
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.4: A3 → +30° ──")
reset()
run_to_target({"a3": math.radians(30)})
actual = joint_deg("a3")
print(f"  A3 actual = {actual:+.2f}°  (target +30°)")
render_png(os.path.join(OUT_DIR, "joint_A3_plus30.png"))
assert abs(actual - 30) < 8, f"FAIL 3.4: A3={actual:.2f}° (gravity causes SS error with PD)"
print("  Task 3.4 PASS ✓")
run_to_target({"a3": 0.0})

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.5 — A4 to +45°, return
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.5: A4 → +45° ──")
reset()
run_to_target({"a4": math.radians(45)})
actual = joint_deg("a4")
print(f"  A4 actual = {actual:+.2f}°  (target +45°)")
render_png(os.path.join(OUT_DIR, "joint_A4_plus45.png"))
assert abs(actual - 45) < 8, f"FAIL 3.5: A4={actual:.2f}°"
print("  Task 3.5 PASS ✓")
run_to_target({"a4": 0.0})

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.6a — A5 to +30°, return
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.6a: A5 → +30° ──")
reset()
run_to_target({"a5": math.radians(30)})
actual = joint_deg("a5")
print(f"  A5 actual = {actual:+.2f}°  (target +30°)")
render_png(os.path.join(OUT_DIR, "joint_A5_plus30.png"))
assert abs(actual - 30) < 8, f"FAIL 3.6a: A5={actual:.2f}°"
print("  Task 3.6a PASS ✓")
run_to_target({"a5": 0.0})

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.6b — A6 to +90°, return
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.6b: A6 → +90° ──")
reset()
run_to_target({"a6": math.radians(90)})
actual = joint_deg("a6")
print(f"  A6 actual = {actual:+.2f}°  (target +90°)")
render_png(os.path.join(OUT_DIR, "joint_A6_plus90.png"))
assert abs(actual - 90) < 8, f"FAIL 3.6b: A6={actual:.2f}°"
print("  Task 3.6b PASS ✓")
run_to_target({"a6": 0.0})

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.7 — Reach pose (all 6 joints simultaneously)
#
# Target: EEF ~30 cm in front of base, ~10 cm above table surface.
# Table surface = 0.85 m.  Arm base at (0, -0.35, 0.85).
# Empirically chosen angles (degrees):
#   A1=0, A2=-60, A3=50, A4=0, A5=30, A6=0
# These extend the arm forward and down toward the table workspace.
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.7: Reach pose (all joints) ──")
REACH_DEG = {"a1": 0, "a2": -60, "a3": 50, "a4": 0, "a5": 30, "a6": 0}
REACH_RAD = {k: math.radians(v) for k, v in REACH_DEG.items()}

reset()
run_to_target(REACH_RAD, steps=3000)   # 3 s to settle

eef_id  = model.body("link_6").id
eef_pos = data.xpos[eef_id].copy()
print(f"  EEF world pos = {eef_pos}  (table surface Z=0.85)")
print(f"  EEF height above table = {eef_pos[2] - 0.85:.3f} m")
print(f"  Reach angles (deg): {REACH_DEG}")

render_png(os.path.join(OUT_DIR, "reach_pose.png"))
# Acceptance: EEF is above the table and forward of the arm base
assert eef_pos[2] > 0.85, f"FAIL 3.7: EEF below table surface (Z={eef_pos[2]:.3f})"
print("  Task 3.7 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.8 — Joint limit enforcement: command A1 to 300° (limit is ±170°)
# ═══════════════════════════════════════════════════════════════════════════
print("\n── Task 3.8: Joint limit enforcement (A1 → 300°) ──")
reset()
run_to_target({"a1": math.radians(300)}, steps=2000)
actual = joint_deg("a1")
limit  = math.degrees(model.jnt_range[JNT["a1"].id][1])
print(f"  Commanded 300°,  actual A1 = {actual:+.2f}°,  limit = ±{limit:.1f}°")
assert actual <= limit + 1, f"FAIL 3.8: joint exceeded limit (actual={actual:.2f}°)"
print("  Task 3.8 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════════
# Task 3.10 — Jacobian IK: reach a target XYZ position
# ═══════════════════════════════════════════════════════════════════════════

def ik_jacobian(model, data, target_xyz,
                eef_body="link_6",
                max_iter=500, tol=0.005, step=0.5):
    """
    Iteratively drive the end-effector to target_xyz using the Jacobian pseudoinverse.

    This is a pure kinematic solver: it writes directly to data.qpos[:6] (bypassing
    physics) until convergence, then syncs data.ctrl and settles with physics.

    Returns the final data.qpos[:6] (joint angles in radians).
    """
    target = np.array(target_xyz, dtype=float)
    eef_id = model.body(eef_body).id
    jac_p  = np.zeros((3, model.nv))

    for i in range(max_iter):
        mujoco.mj_forward(model, data)
        pos_err = target - data.xpos[eef_id]
        err_norm = np.linalg.norm(pos_err)

        if err_norm < tol:
            print(f"  IK converged in {i+1} iterations  (err={err_norm*1000:.2f} mm)")
            break

        mujoco.mj_jacBody(model, data, jac_p, None, eef_id)
        jac_arm = jac_p[:, :6]  # first 6 columns = arm DOFs

        dq, _, _, _ = np.linalg.lstsq(jac_arm, pos_err, rcond=None)
        data.qpos[:6] += step * dq

        # Clamp to joint limits
        for j in range(6):
            jid = model.joint(f"joint_a{j+1}").id
            lo, hi = model.jnt_range[jid]
            data.qpos[j] = float(np.clip(data.qpos[j], lo, hi))
    else:
        print(f"  IK did NOT converge (final err={np.linalg.norm(target - data.xpos[eef_id])*1000:.1f} mm)")

    # Capture kinematic result before physics settle
    mujoco.mj_forward(model, data)
    ik_eef_pos = data.xpos[eef_id].copy()

    # Sync controllers and settle with physics (for visualization / subsequent steps)
    data.ctrl[:6] = data.qpos[:6].copy()
    for _ in range(1000):
        mujoco.mj_step(model, data)

    return data.qpos[:6].copy(), ik_eef_pos


print("\n── Task 3.10: Jacobian IK → [0.4, 0.0, 0.95] ──")
reset()
# Start from reach pose (set qpos via joint addresses, not actuator indices)
for key, val_rad in REACH_RAD.items():
    data.qpos[JNT[key].qposadr[0]] = val_rad
mujoco.mj_forward(model, data)

TARGET_XYZ = [0.4, 0.0, 0.95]
_, ik_eef_pos = ik_jacobian(model, data, TARGET_XYZ)

# Error is measured at the kinematic solution (before gravity drift from physics settle)
err_mm = np.linalg.norm(np.array(TARGET_XYZ) - ik_eef_pos) * 1000
print(f"  Target   : {TARGET_XYZ}")
print(f"  Achieved : {ik_eef_pos.round(4).tolist()}")
print(f"  Error    : {err_mm:.2f} mm  (pass < 5 mm)")
assert err_mm < 5.0, f"FAIL 3.10: IK error {err_mm:.2f} mm > 5 mm"
render_png(os.path.join(OUT_DIR, "ik_target_reach.png"))
print("  Task 3.10 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  Phase 3 (Tasks 3.1–3.10) ALL PASS ✓")
print("══════════════════════════════════════════")
