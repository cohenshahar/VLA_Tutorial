"""
Task 0.6 + 0.7 — Verify MuJoCo works and viewer opens.

Task 0.6: Load a self-contained model, run 100 steps, print simulation time.
Task 0.7: Open the passive viewer so the model can be viewed interactively.

Run: python test_mujoco.py
Close the viewer window (Esc or X) to exit.
"""
import os
import gymnasium
import mujoco
import mujoco.viewer

# --- Task 0.6: use a self-contained model from gymnasium (no external meshes needed) ---
# gymnasium's CartPole / Hopper / etc assets are single-file XML suitable for this test
_gym_assets = os.path.join(os.path.dirname(gymnasium.__file__), "envs", "mujoco", "assets")
model_path = os.path.join(_gym_assets, "hopper.xml")  # single-file, no separate meshes
print(f"Loading model: {model_path}")

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

for _ in range(100):
    mujoco.mj_step(model, data)

print(f"Simulation time after 100 steps: {data.time:.4f} s")  # expected ~0.1 s

# --- Task 0.7: open passive viewer ---
print("Opening MuJoCo passive viewer — close the window (Esc or X) to exit.")
with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()

print("Viewer closed.")
