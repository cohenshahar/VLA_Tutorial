"""
scene/view_scene.py — Open the full scene with a good default camera.

Run from VLATraining/sim/:
    python -m scene.view_scene

Camera is set programmatically before the first frame so you always get
the same comfortable view angle — front-right diagonal, arm + table centred.
"""
import math
import os
import mujoco
import mujoco.viewer

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")

model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

# Apply gains and armature so the arm holds its pose
_GAINS = {
    "act_a1": (800, 80), "act_a2": (800, 80), "act_a3": (500, 50),
    "act_a4": (300, 30), "act_a5": (500, 50), "act_a6": (300, 30),
}
for name, (kp, kv) in _GAINS.items():
    ai = model.actuator(name).id
    model.actuator_gainprm[ai, 0] =  kp
    model.actuator_biasprm[ai, 1] = -kp
    model.actuator_biasprm[ai, 2] = -kv
for i in range(model.nv):
    model.dof_armature[i] = 0.5

# Settle physics (box lands on table, arm holds home pose)
for _ in range(500):
    mujoco.mj_step(model, data)

with mujoco.viewer.launch_passive(model, data) as v:
    # ── Good default camera ──────────────────────────────────────────
    # lookat: centre of table workspace (arm base at Y=-0.35, box at Y≈0)
    # azimuth 140°: front-right diagonal view
    # elevation -20°: slightly above, looking down
    # distance 3.2 m: whole scene fits comfortably
    v.cam.lookat[:] = [0.0, 0.1, 0.9]
    v.cam.azimuth   = 140
    v.cam.elevation = -20
    v.cam.distance  = 3.2
    v.sync()

    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
