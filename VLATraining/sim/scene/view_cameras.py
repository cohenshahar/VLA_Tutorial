"""
view_cameras.py — Open MuJoCo viewer with camera direction arrows overlaid.

Arrows:
  RED   = cam_overhead
  GREEN = cam_side
  BLUE  = cam_wrist

The arrow starts at the camera position and points 0.6 m along the camera's
optical axis (local +Z).  A small sphere marks the camera origin.

Run:
  source /home/shahar/Desktop/phase4/phase4_env/bin/activate
  cd /home/shahar/Desktop/phase4/VLATraining/sim
  PYTHONPATH=. python scene/view_cameras.py
"""
import time
import mujoco
import mujoco.viewer
import numpy as np
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from arm.load_arm import apply_gains

# ── load model ──────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path('scene/world.xml')
data  = mujoco.MjData(model)
apply_gains(model, {
    'act_a1': (800, 80), 'act_a2': (800, 80), 'act_a3': (500, 50),
    'act_a4': (300, 30), 'act_a5': (500, 50), 'act_a6': (300, 30),
})

# box on table, arm in snap pose
box_jid = model.joint('box_freejoint').id
bqpa    = model.jnt_qposadr[box_jid]
data.qpos[bqpa:bqpa+7] = [0.30, 0.0, 0.90, 1, 0, 0, 0]

snap_q = np.array([0.0, -0.96, 1.57, 0.0, 0.96, 0.0])
for i, n in enumerate([f'joint_a{j}' for j in range(1, 7)]):
    data.qpos[model.jnt_qposadr[model.joint(n).id]] = snap_q[i]

mujoco.mj_forward(model, data)

# ── camera spec ─────────────────────────────────────────────────────────────
cam_names = ['cam_overhead', 'cam_side', 'cam_wrist']
cam_labels = ['OVERHEAD (red)', 'SIDE (green)', 'WRIST (blue)']
colors = [
    np.array([1.0, 0.1, 0.1, 1.0], dtype=np.float32),   # red
    np.array([0.1, 0.9, 0.1, 1.0], dtype=np.float32),   # green
    np.array([0.2, 0.5, 1.0, 1.0], dtype=np.float32),   # blue
]
ARROW_LEN = 0.6   # metres
ARROW_W   = 0.015
SPHERE_R  = 0.025

# ── print camera info ────────────────────────────────────────────────────────
print("\nCamera positions and optical axes (world frame):")
print("-" * 55)
for name, label in zip(cam_names, cam_labels):
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    pos = data.cam_xpos[cid]
    mat = data.cam_xmat[cid].reshape(3, 3)
    axis = mat[:, 2]           # camera +Z = optical axis
    print(f"  {label}")
    print(f"    pos  = [{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")
    print(f"    axis = [{axis[0]:+.3f}, {axis[1]:+.3f}, {axis[2]:+.3f}]")
print("-" * 55)
print("Close the viewer window to exit.\n")

# ── launch viewer ────────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    # zoom out a bit so cameras are visible
    viewer.cam.distance = 4.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth   = 135

    while viewer.is_running():
        mujoco.mj_forward(model, data)
        viewer.user_scn.ngeom = 0

        for cam_name, color in zip(cam_names, colors):
            cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
            pos  = data.cam_xpos[cid].copy()
            mat  = data.cam_xmat[cid].reshape(3, 3)
            axis = mat[:, 2]           # camera +Z
            tip  = pos + axis * ARROW_LEN
            n    = viewer.user_scn.ngeom

            # ── arrow shaft ─────────────────────────────────────────────────
            if n < viewer.user_scn.maxgeom:
                g = viewer.user_scn.geoms[n]
                mujoco.mjv_initGeom(
                    g, mujoco.mjtGeom.mjGEOM_ARROW,
                    np.zeros(3), np.zeros(3), np.zeros(9), color,
                )
                mujoco.mjv_connector(
                    g, mujoco.mjtGeom.mjGEOM_ARROW, ARROW_W, pos, tip,
                )
                viewer.user_scn.ngeom += 1
                n += 1

            # ── sphere at camera origin ──────────────────────────────────────
            if n < viewer.user_scn.maxgeom:
                g = viewer.user_scn.geoms[n]
                mujoco.mjv_initGeom(
                    g, mujoco.mjtGeom.mjGEOM_SPHERE,
                    pos, np.array([SPHERE_R, SPHERE_R, SPHERE_R]),
                    np.eye(3).flatten(), color,
                )
                viewer.user_scn.ngeom += 1

        viewer.sync()
        time.sleep(0.033)
