"""
Phase 2 viewer demo — arm motion + table collision.

Run from VLATraining/sim/:
    python -m scene.demo_viewer

What you will see:
  1. Arm sweeps joint A2 downward toward the table (negative angle).
  2. The arm link makes contact with the table surface and STOPS —
     it cannot pass through because MuJoCo resolves mesh-vs-box collision.
  3. Arm then lifts back up, and repeats.

Controls in the viewer window (standard MuJoCo):
  - Left-drag   : orbit
  - Right-drag  : pan
  - Scroll      : zoom
  - Space       : pause / resume
  - Esc         : quit
"""
import math
import mujoco
import mujoco.viewer
import numpy as np
import os

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")

model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)
print(f"Loaded  nq={model.nq}  nu={model.nu}  ngeom={model.ngeom}")
print("Opening viewer — use mouse to orbit, Space to pause, Esc to quit.")

# ── Motion parameters ────────────────────────────────────────────────────
# A2 sweeps from 0 → -1.3 rad (~-75°, toward table) and back.
# Joint limit for A2 is [-3.31, +0.785] so -1.3 is well within range.
# At A2 = -1.3 the lower links reach toward the table; collision stops them.
A2_DOWN  =  math.radians(-75)   # target when sweeping down
A2_UP    =  math.radians(0)     # home position
PERIOD   = 4.0                  # seconds per half-sweep
HOLD     = 0.5                  # seconds to hold at each extreme

# ── Viewer loop ──────────────────────────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    sim_time    = 0.0
    phase_start = 0.0
    phase       = "down"   # "down" | "hold_down" | "up" | "hold_up"

    while viewer.is_running():
        elapsed = sim_time - phase_start

        if phase == "down":
            t       = min(elapsed / PERIOD, 1.0)
            target  = A2_UP + t * (A2_DOWN - A2_UP)
            data.ctrl[1] = target          # act_a2
            if elapsed >= PERIOD:
                phase = "hold_down"
                phase_start = sim_time

        elif phase == "hold_down":
            data.ctrl[1] = A2_DOWN
            if elapsed >= HOLD:
                phase = "up"
                phase_start = sim_time

        elif phase == "up":
            t       = min(elapsed / PERIOD, 1.0)
            target  = A2_DOWN + t * (A2_UP - A2_DOWN)
            data.ctrl[1] = target
            if elapsed >= PERIOD:
                phase = "hold_up"
                phase_start = sim_time

        elif phase == "hold_up":
            data.ctrl[1] = A2_UP
            if elapsed >= HOLD:
                phase = "down"
                phase_start = sim_time

        mujoco.mj_step(model, data)
        sim_time += model.opt.timestep

        # Print contact info every 0.5 s
        if abs(sim_time % 0.5) < model.opt.timestep:
            ncon = data.ncon
            a2   = math.degrees(data.qpos[model.joint("joint_a2").qposadr[0]])
            print(f"t={sim_time:5.2f}s  phase={phase:10s}  A2={a2:+7.2f}°  contacts={ncon}")

        viewer.sync()
