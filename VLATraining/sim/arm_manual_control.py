"""
POC — Manual joint control for KUKA KR6 R900 sixx.

- No gravity
- No joint limits
- Opens MuJoCo interactive viewer with joint sliders

Controls (in the viewer window):
  Ctrl+J  — toggle the Joints panel (drag sliders to move each joint)
  Mouse   — orbit / zoom / pan
  Esc     — close

Run:
    python arm_manual_control.py
"""
import os
import numpy as np
import mujoco
import mujoco.viewer

URDF_PATH = os.path.join(
    os.path.dirname(__file__),
    "assets", "urdf", "kr6r900sixx.urdf"
)

model = mujoco.MjModel.from_xml_path(URDF_PATH)
data = mujoco.MjData(model)

# ── No gravity ──
model.opt.gravity[:] = 0.0

# ── No joint limits ──
model.jnt_limited[:] = 0

# ── Framebuffer for any offscreen use ──
model.vis.global_.offwidth = 1280
model.vis.global_.offheight = 720

print("Opening interactive viewer...")
print("  Press Ctrl+J to open the Joints panel")
print("  Drag the sliders to move each joint")
print("  Press Esc or close the window to exit")
print()
print("Joints available:")
for i in range(model.njnt):
    jnt = model.joint(i)
    jtype = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(int(jnt.type[0]), "?")
    if jtype == "hinge":
        print(f"  {jnt.name}")

# launch() is the full interactive viewer — blocks until window is closed
mujoco.viewer.launch(model, data)
print("Viewer closed.")
