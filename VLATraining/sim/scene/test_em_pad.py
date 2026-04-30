"""
Phase 4 — EM End-Effector verification (Tasks 4.1–4.4).

Run from VLATraining/sim/:
    python -m scene.test_em_pad

What this script does:
  4.1  Verify em_pad body exists and moves with the arm
  4.2  Verify em_contact_site can be looked up without error
  4.3  Print em_contact_site world position at home vs reach pose (must differ)
  4.4  Render front view and top-down view of arm in reach pose
       → outputs/em_pad_attached.png
       → outputs/em_pad_frontview.png
       → outputs/em_pad_topview.png
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

# ── Paths ─────────────────────────────────────────────────────────────────
SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR = os.path.join(SIM_DIR, "outputs")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Load model ─────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

# Runtime gains (same as test_joints.py)
_GAINS = {
    "act_a1": (800.0, 80.0),
    "act_a2": (800.0, 80.0),
    "act_a3": (500.0, 50.0),
    "act_a4": (300.0, 30.0),
    "act_a5": (500.0, 50.0),
    "act_a6": (300.0, 30.0),
}
for name, (kp, kv) in _GAINS.items():
    ai = model.actuator(name).id
    model.actuator_gainprm[ai, 0] =  kp
    model.actuator_biasprm[ai, 1] = -kp
    model.actuator_biasprm[ai, 2] = -kv
for i in range(model.nv):
    model.dof_armature[i] = 0.5

REACH_DEG = {"a1": 0, "a2": -60, "a3": 50, "a4": 0, "a5": 30, "a6": 0}
REACH_RAD = {k: math.radians(v) for k, v in REACH_DEG.items()}

EM_SITE_ID  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "em_contact_site")
BOX_SITE_ID = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "box_top_site")
EM_PAD_ID   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "em_pad")


def _make_cam(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance  = distance
    cam.azimuth   = azimuth
    cam.elevation = elevation
    return cam


def render_png(path, cam):
    with mujoco.Renderer(model, height=720, width=1280) as r:
        r.update_scene(data, camera=cam)
        _save(r.render(), path)
    print(f"  Saved → {path}")


# ═════════════════════════════════════════════════════════════════════════
# Task 4.2 — Verify em_contact_site exists
# ═════════════════════════════════════════════════════════════════════════
print("\n── Task 4.2: em_contact_site lookup ──")
assert EM_SITE_ID >= 0, "FAIL: em_contact_site not found in model"
assert BOX_SITE_ID >= 0, "FAIL: box_top_site not found in model"
assert EM_PAD_ID >= 0, "FAIL: em_pad body not found in model"
print(f"  em_contact_site id  = {EM_SITE_ID}  ✓")
print(f"  box_top_site id     = {BOX_SITE_ID}  ✓")
print(f"  em_pad body id      = {EM_PAD_ID}  ✓")
print("  Task 4.2 PASS ✓")

# ═════════════════════════════════════════════════════════════════════════
# Task 4.3 — em_contact_site moves with the arm (home vs reach pose)
# ═════════════════════════════════════════════════════════════════════════
print("\n── Task 4.3: em_contact_site moves with arm ──")

# Home pose (all joints = 0)
mujoco.mj_resetData(model, data)
data.qpos[:] = 0.0
data.qvel[:] = 0.0
mujoco.mj_forward(model, data)
home_pos = data.site_xpos[EM_SITE_ID].copy()
print(f"  Home pose  em_contact_site = {home_pos}")

# Reach pose (run physics to settle)
mujoco.mj_resetData(model, data)
data.qpos[:] = 0.0
data.ctrl[:] = 0.0
for jname, rad in REACH_RAD.items():
    data.ctrl[model.actuator("act_" + jname).id] = rad
for _ in range(3000):
    mujoco.mj_step(model, data)
reach_pos = data.site_xpos[EM_SITE_ID].copy()
print(f"  Reach pose em_contact_site = {reach_pos}")

displacement = float(np.linalg.norm(reach_pos - home_pos))
print(f"  Displacement = {displacement:.4f} m")
assert displacement > 0.05, f"FAIL 4.3: em_contact_site barely moved ({displacement:.4f} m)"
print("  Task 4.3 PASS ✓")

# ═════════════════════════════════════════════════════════════════════════
# Task 4.1 — Render showing EM pad attached to arm
# ═════════════════════════════════════════════════════════════════════════
print("\n── Task 4.1/4.4: Rendering EM pad views ──")

# Wide front-diagonal view (arm in reach pose, full scene visible)
cam_front = _make_cam(lookat=[0.0, 0.1, 1.0], distance=2.5, azimuth=140, elevation=-20)
render_png(os.path.join(OUT_DIR, "em_pad_attached.png"), cam_front)

# Front view (side-on)
cam_fv = _make_cam(lookat=[0.0, 0.1, 1.0], distance=2.2, azimuth=180, elevation=-15)
render_png(os.path.join(OUT_DIR, "em_pad_frontview.png"), cam_fv)

# Top-down view
cam_top = _make_cam(lookat=[0.0, 0.1, 1.0], distance=2.0, azimuth=140, elevation=-80)
render_png(os.path.join(OUT_DIR, "em_pad_topview.png"), cam_top)

# ═════════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════")
print("  Phase 4 (Tasks 4.1–4.4) ALL PASS ✓")
print("══════════════════════════════════════════")
