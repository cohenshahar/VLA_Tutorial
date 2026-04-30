"""
Task 2.1 / 2.2 / 2.3 render verification — load scene/world.xml and save PNGs.

Run from VLATraining/sim/:
    python -m scene.render_21

Saves:
  outputs/scene_world_21.png        — arm on floor (2.1)
  outputs/scene_arm_on_table.png    — arm mounted on table (2.2 / 2.3)
"""
import os
import mujoco
import numpy as np

try:
    from PIL import Image
    _HAS_PIL = True
except ImportError:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAS_PIL = False

SIM_DIR  = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD    = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR  = os.path.join(SIM_DIR, "outputs")
os.makedirs(OUT_DIR, exist_ok=True)


def save_png(pixels: np.ndarray, path: str) -> None:
    if _HAS_PIL:
        Image.fromarray(pixels).save(path)
    else:
        plt.imsave(path, pixels)
    print(f"Saved → {path}")


def render_scene(lookat, distance, azimuth, elevation) -> np.ndarray:
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance  = distance
    cam.azimuth   = azimuth
    cam.elevation = elevation
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera=cam)
        return renderer.render()


# ── Load model ───────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)
print(f"Loaded  nq={model.nq}  nbody={model.nbody}  ngeom={model.ngeom}  nlight={model.nlight}")

# Framebuffer fix (must be before Renderer)
model.vis.global_.offwidth  = 1280
model.vis.global_.offheight = 720

mujoco.mj_forward(model, data)

# ── Render 1: wide view showing arm + table ───────────────────────────────
pixels = render_scene(
    lookat   = [0.0, 0.1, 1.1],
    distance = 2.8,
    azimuth  = 140,
    elevation= -18,
)
save_png(pixels, os.path.join(OUT_DIR, "scene_arm_on_table.png"))

# ── Render 2: close-up showing arm base flush on table ───────────────────
pixels2 = render_scene(
    lookat   = [0.0, -0.35, 0.9],
    distance = 1.0,
    azimuth  = 180,
    elevation= -20,
)
save_png(pixels2, os.path.join(OUT_DIR, "scene_arm_base_closeup.png"))

print("Task 2.2 / 2.3 render complete ✓")
