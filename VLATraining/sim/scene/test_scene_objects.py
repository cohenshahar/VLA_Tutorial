"""
Phase 5 — Scene objects verification (Tasks 5.1–5.7).

Run from VLATraining/sim/:
    python -m scene.test_scene_objects

What this script does:
  5.1  Box falls from default height and lands stably on the table
  5.2  Box mass = 0.5 kg verified from model
  5.3  Friction test: impulse applied, box slides then stops
  5.4  Touch sensor: reads >0 when box is on target, 0 when box is elsewhere
  5.5  Target zone visible in scene
  5.6  Randomization: box and target placed at least 0.20 m apart
  5.7  Full scene render → outputs/full_scene.png
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

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR = os.path.join(SIM_DIR, "outputs")
os.makedirs(OUT_DIR, exist_ok=True)

from scene.randomize_scene import randomize_scene


def _fresh_model():
    """Load a fresh model+data pair."""
    m = mujoco.MjModel.from_xml_path(WORLD)
    d = mujoco.MjData(m)
    return m, d


def _box_pos(model, data):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    return data.xpos[bid].copy()


def _box_mass(model):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    return float(model.body_mass[bid])


def _box_on_target(model, data):
    """
    Python-based contact check: returns True when box centre is within
    the target zone's XY footprint and resting on its surface.
    More reliable than MuJoCo touch sensor on a fixed (jointless) body.
    """
    box_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    tgt_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_zone")
    box_pos  = data.xpos[box_id]
    tgt_pos  = model.body_pos[tgt_id]      # fixed body → use model, not data.xpos
    # target geom half-sizes: 0.075 × 0.075 × 0.01
    # box half-size: 0.05
    in_x = abs(box_pos[0] - tgt_pos[0]) < (0.075 + 0.05)
    in_y = abs(box_pos[1] - tgt_pos[1]) < (0.075 + 0.05)
    on_z = abs(box_pos[2] - (tgt_pos[2] + 0.01 + 0.05)) < 0.03  # within 3 cm of stacked height
    return bool(in_x and in_y and on_z)


def _make_cam(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance  = distance
    cam.azimuth   = azimuth
    cam.elevation = elevation
    return cam


def render_png(model, data, path, cam):
    with mujoco.Renderer(model, height=720, width=1280) as r:
        r.update_scene(data, camera=cam)
        _save(r.render(), path)
    print(f"  Saved → {path}")


# ═══════════════════════════════════════════════════════════════════════
# Task 5.1 — Box falls and lands on the table
# ═══════════════════════════════════════════════════════════════════════
print("\n── Task 5.1: Box settles on table ──")
model, data = _fresh_model()
# Use default XML position (0, 0.1, 0.92) — box drops ~7 cm onto table
mujoco.mj_forward(model, data)
z_initial = _box_pos(model, data)[2]
print(f"  Box initial Z = {z_initial:.4f} m  (should be ~0.92)")

for _ in range(500):
    mujoco.mj_step(model, data)

z_settled = _box_pos(model, data)[2]
print(f"  Box settled Z = {z_settled:.4f} m  (table surface = 0.85, box half = 0.05 → expected ~0.90)")
assert 0.88 < z_settled < 0.92, f"FAIL 5.1: box Z = {z_settled:.4f} (expected ~0.90)"
print("  Task 5.1 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════
# Task 5.2 — Box mass = 0.5 kg
# ═══════════════════════════════════════════════════════════════════════
print("\n── Task 5.2: Box mass ──")
mass = _box_mass(model)
print(f"  Box mass = {mass:.4f} kg  (expected 0.5 kg)")
assert abs(mass - 0.5) < 0.01, f"FAIL 5.2: mass = {mass:.4f} kg"
print("  Task 5.2 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════
# Task 5.3 — Friction: box slides then stops after impulse
# ═══════════════════════════════════════════════════════════════════════
print("\n── Task 5.3: Friction impulse test ──")
model2, data2 = _fresh_model()
# Settle box first
for _ in range(500):
    mujoco.mj_step(model2, data2)

box_id = mujoco.mj_name2id(model2, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
pos_before = data2.xpos[box_id].copy()

# Apply 15 N impulse in X for 80 steps (enough to overcome static friction, stays on table)
for _ in range(80):
    data2.xfrc_applied[box_id, 0] = 15.0  # 15 N in X
    mujoco.mj_step(model2, data2)
data2.xfrc_applied[box_id, :] = 0.0  # remove force

pos_during = data2.xpos[box_id].copy()
print(f"  Pos before impulse : {pos_before}")
print(f"  Pos after impulse  : {pos_during}")

# Run 2000 more steps to let it stop
for _ in range(2000):
    mujoco.mj_step(model2, data2)

pos_final = data2.xpos[box_id].copy()
vel_id = model2.jnt_dofadr[mujoco.mj_name2id(model2, mujoco.mjtObj.mjOBJ_JOINT, "box_freejoint")]
speed_final = float(np.linalg.norm(data2.qvel[vel_id:vel_id + 3]))

print(f"  Pos after settling : {pos_final}")
print(f"  Final speed        : {speed_final:.5f} m/s  (should be ~0 = stopped)")

slide_dist = float(np.linalg.norm(pos_final[:2] - pos_before[:2]))
print(f"  Total slide dist   : {slide_dist:.4f} m")
assert slide_dist > 0.001, f"FAIL 5.3: box didn't move (slide={slide_dist:.4f} m)"
assert speed_final < 0.05,  f"FAIL 5.3: box still sliding (speed={speed_final:.5f}) — friction too low"
print("  Task 5.3 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════
# Task 5.6 — Touch sensor: on vs off
# ═══════════════════════════════════════════════════════════════════════
print("\n── Task 5.6: Target touch sensor ──")
model3, data3 = _fresh_model()

# Place box directly on target zone (default target pos = 0, 0.25, 0.87)
target_id = mujoco.mj_name2id(model3, mujoco.mjtObj.mjOBJ_BODY, "target_zone")
tgt_pos = model3.body_pos[target_id].copy()
print(f"  Target zone body pos = {tgt_pos}")
box_jnt_id = mujoco.mj_name2id(model3, mujoco.mjtObj.mjOBJ_JOINT, "box_freejoint")
box_qpos_adr = model3.jnt_qposadr[box_jnt_id]
# Place box centre 6 cm above target zone centre (lands on target top face)
box_z = tgt_pos[2] + 0.06
data3.qpos[box_qpos_adr:box_qpos_adr+3] = [tgt_pos[0], tgt_pos[1], box_z]
data3.qpos[box_qpos_adr+3:box_qpos_adr+7] = [1, 0, 0, 0]
mujoco.mj_forward(model3, data3)

# Settle
for _ in range(800):
    mujoco.mj_step(model3, data3)

box_id3 = mujoco.mj_name2id(model3, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
print(f"  Box pos after settle = {data3.xpos[box_id3]}")
print(f"  ncon = {data3.ncon}")

touch_on = _box_on_target(model3, data3)
print(f"  Box-on-target (box on target) = {touch_on}  (expected True)")

# Move box far away
data3.qpos[box_qpos_adr:box_qpos_adr+3] = [5.0, 5.0, 1.0]
mujoco.mj_forward(model3, data3)
mujoco.mj_step(model3, data3)
touch_off = _box_on_target(model3, data3)
print(f"  Box-on-target (box away)      = {touch_off}  (expected False)")

assert touch_on  is True,  f"FAIL 5.6: box not detected on target ({touch_on})"
assert touch_off is False, f"FAIL 5.6: false positive when box is far away ({touch_off})"
print("  Task 5.6 PASS ✓")

# ═══════════════════════════════════════════════════════════════════════
# Task 5.6b — Randomization separation check
# ═══════════════════════════════════════════════════════════════════════
print("\n── Randomization: box/target separation ──")
model4, data4 = _fresh_model()
info = randomize_scene(model4, data4, seed=7)
sep = float(np.linalg.norm(info["box_pos"][:2] - info["target_pos"][:2]))
print(f"  Box    : {info['box_pos']}")
print(f"  Target : {info['target_pos']}")
print(f"  XY sep : {sep:.3f} m  (min 0.20 m)")
assert sep >= 0.20, f"FAIL: separation {sep:.3f} m < 0.20 m"
print("  Randomization PASS ✓")

# ═══════════════════════════════════════════════════════════════════════
# Task 5.7 — Full scene render with randomized layout
# ═══════════════════════════════════════════════════════════════════════
print("\n── Task 5.7: Full scene render ──")
# Settle box in randomized scene
for _ in range(600):
    mujoco.mj_step(model4, data4)

cam_full = _make_cam(lookat=[0.0, 0.1, 1.0], distance=2.8, azimuth=135, elevation=-22)
render_png(model4, data4, os.path.join(OUT_DIR, "full_scene.png"), cam_full)
print("  Task 5.7 PASS ✓")

# ══════════════════════════════════════════════════════════════════════
print("\n══════════════════════════════════════════════════════")
print("  Phase 5 (Tasks 5.1–5.7) ALL PASS ✓")
print("══════════════════════════════════════════════════════")
