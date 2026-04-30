"""
Task 2.5 — Physics step-loop verification for scene/world.xml.

Run from VLATraining/sim/:
    python -m scene.test_world

Done when:
  - Prints 30 lines of timestamps (0.1 s … 3.0 s)
  - End-effector Z stays stable (no drift / explosion)
  - Viewer opens showing the arm in its resting pose
"""
import math
import mujoco
import mujoco.viewer
import numpy as np
import os

SIM_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD    = os.path.join(SIM_DIR, "scene", "world.xml")

model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

# End-effector body name (last link in kinematic chain)
EEF_BODY = "link_6"
eef_id   = model.body(EEF_BODY).id

print(f"Loaded scene/world.xml  nq={model.nq}  timestep={model.opt.timestep}")
print(f"Simulating 3000 steps (3.0 s)…\n")
print(f"{'Time (s)':>10}  {'EEF Z (m)':>12}  {'qpos norm':>12}")
print("-" * 40)

for step in range(3000):
    mujoco.mj_step(model, data)

    if (step + 1) % 100 == 0:
        eef_z    = data.xpos[eef_id, 2]
        qpos_ok  = np.linalg.norm(data.qpos)
        print(f"{data.time:>10.3f}  {eef_z:>12.4f}  {qpos_ok:>12.4f}")

has_nan = np.any(np.isnan(data.qpos))
print(f"\nqpos contains NaN: {has_nan}")
print("Step-loop PASS ✓" if not has_nan else "FAIL — NaN detected")

# ── Open passive viewer ───────────────────────────────────────────────────
print("\nOpening viewer — close window to exit.")
with mujoco.viewer.launch_passive(model, data) as v:
    v.cam.lookat[:] = [0.0, 0.1, 0.9]
    v.cam.azimuth   = 140
    v.cam.elevation = -20
    v.cam.distance  = 3.2
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
