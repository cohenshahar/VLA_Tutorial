"""
Phase 7, Task 7.6 — Sensor data visualisation.

Runs a full pick-carry-drop sequence, logs all sensor channels via
SensorLogger, then produces a 4-panel matplotlib figure saved to
outputs/sensor_plot.png.

Run from VLATraining/sim/:
    python -m scene.plot_sensors
"""
import math, os, sys
import mujoco
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR = os.path.join(SIM_DIR, "outputs")
os.makedirs(OUT_DIR, exist_ok=True)
sys.path.insert(0, SIM_DIR)

from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate
from scene.sensor_logger import SensorLogger

# ── Load ──────────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}
apply_gains(model, _GAINS)

SNAP  = {"a1":  0, "a2": -55, "a3": 90, "a4": 0, "a5": 55, "a6": 0}
CARRY = {"a1": 45, "a2": -55, "a3": 90, "a4": 0, "a5": 55, "a6": 0}
ACT   = {f"a{i+1}": model.actuator(f"act_a{i+1}").id for i in range(6)}

em_site_id  = model.site("em_contact_site").id
box_bid     = model.body("metal_box").id
box_jnt_id  = model.joint("box_freejoint").id
box_qpa     = model.jnt_qposadr[box_jnt_id]
box_doa     = model.jnt_dofadr[box_jnt_id]

# ── Step 1: find snap-pose EM position ────────────────────────────────────
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)
for _ in range(3000):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
em_snap = data.site_xpos[em_site_id].copy()

# ── Step 2: reset + place box ─────────────────────────────────────────────
mujoco.mj_resetData(model, data)
apply_gains(model, _GAINS)
BOX_HALF, GAP = 0.05, 0.015
box_z0   = em_snap[2] - BOX_HALF - GAP
box_init = np.array([em_snap[0], em_snap[1], box_z0, 1., 0., 0., 0.])
data.qpos[box_qpa:box_qpa+7] = box_init
mujoco.mj_forward(model, data)
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)

# ── Step 3: converge arm, box pinned ─────────────────────────────────────
for _ in range(3000):
    data.qpos[box_qpa:box_qpa+7] = box_init
    data.qvel[box_doa:box_doa+6] = 0.0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
data.qvel[:] = 0.0
data.qpos[box_qpa:box_qpa+7] = box_init
mujoco.mj_forward(model, data)

# ── Step 4: activate weld ─────────────────────────────────────────────────
ok = em_activate(model, data, threshold_m=0.05)
assert ok, "em_activate failed"

# ── Step 5: settling (500 steps, not logged) ──────────────────────────────
for _ in range(500):
    mujoco.mj_step(model, data)

# ── Phase A: hold at snap pose (1000 steps) ───────────────────────────────
csv_path = os.path.join(OUT_DIR, "sensor_log.csv")
logger = SensorLogger(model, csv_path)

PHASE_HOLD  = 1000   # hold at snap (weld active, arm still)
PHASE_CARRY = 1500   # move to carry pose
PHASE_DROP  = 800    # after deactivation

t_weld_on   = data.time
t_carry_cmd = None
t_weld_off  = None

for step in range(PHASE_HOLD):
    mujoco.mj_step(model, data)
    logger.step(data)

# ── Phase B: set carry pose ctrl + simulate ───────────────────────────────
t_carry_cmd = data.time
for name, deg in CARRY.items():
    data.ctrl[ACT[name]] = math.radians(deg)
for step in range(PHASE_CARRY):
    mujoco.mj_step(model, data)
    logger.step(data)

# ── Phase C: deactivate weld + drop ──────────────────────────────────────
t_weld_off = data.time
em_deactivate(model, data)
for step in range(PHASE_DROP):
    mujoco.mj_step(model, data)
    logger.step(data)

logger.finish()
print(f"Logged {logger.rows_written} rows → {csv_path}")

# ── Load CSV ──────────────────────────────────────────────────────────────
import csv as _csv
with open(csv_path) as f:
    reader = _csv.DictReader(f)
    rows   = list(reader)

def col(name):
    return np.array([float(r[name]) for r in rows])

t = col("t")

# ── Build figure (4 panels) ───────────────────────────────────────────────
JOINT_COLORS = ["#e41a1c","#377eb8","#4daf4a","#984ea3","#ff7f00","#a65628"]

fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Phase 7 — Sensor Suite  |  Pick → Carry → Drop", fontsize=14, fontweight="bold")

# Vertical phase markers
def phase_lines(ax):
    ax.axvline(t_carry_cmd, color="steelblue",  ls="--", lw=1.2, alpha=0.7, label="carry cmd")
    ax.axvline(t_weld_off,  color="firebrick",  ls="--", lw=1.2, alpha=0.7, label="weld off")

# ── Panel 1: Joint positions ──────────────────────────────────────────────
ax = axes[0]
for i in range(1, 7):
    vals = np.degrees(col(f"pos_A{i}"))
    ax.plot(t, vals, color=JOINT_COLORS[i-1], lw=1.4, label=f"A{i}")
phase_lines(ax)
ax.set_ylabel("Joint position (°)")
ax.set_title("7.1  Joint Positions")
ax.legend(ncol=6, fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ── Panel 2: Joint velocities ─────────────────────────────────────────────
ax = axes[1]
for i in range(1, 7):
    vals = col(f"vel_A{i}")
    ax.plot(t, vals, color=JOINT_COLORS[i-1], lw=1.2, label=f"A{i}")
phase_lines(ax)
ax.set_ylabel("Vel (rad/s)")
ax.set_title("7.2  Joint Velocities")
ax.legend(ncol=6, fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ── Panel 3: Actuator torques ─────────────────────────────────────────────
ax = axes[2]
for i in range(1, 7):
    vals = col(f"torque_A{i}")
    ax.plot(t, vals, color=JOINT_COLORS[i-1], lw=1.4, label=f"A{i}")
phase_lines(ax)
ax.set_ylabel("Torque (N·m)")
ax.set_title("7.3  Actuator Torques")
ax.legend(ncol=6, fontsize=7, loc="upper left")
ax.grid(True, alpha=0.3)

# ── Panel 4: EE force magnitude + proximity ───────────────────────────────
ax = axes[3]
ee_fx = col("ee_fx"); ee_fy = col("ee_fy"); ee_fz = col("ee_fz")
ee_mag = np.sqrt(ee_fx**2 + ee_fy**2 + ee_fz**2)
prox   = col("proximity")

ax2 = ax.twinx()
ax.plot(t, ee_mag, color="darkorange",  lw=1.8, label="|EE force| (N)")
ax.axhline(0.5 * 9.81, color="darkorange", ls=":", lw=1.2, alpha=0.6, label="box weight 4.9 N")
ax2.plot(t, prox,   color="teal",        lw=1.4, ls="-", alpha=0.7, label="proximity (m)")
phase_lines(ax)
ax.set_ylabel("|EE force| (N)", color="darkorange")
ax2.set_ylabel("Proximity (m)", color="teal")
ax.set_title("7.4 / 7.5  EE Force Magnitude + Proximity")
ax.set_xlabel("Simulation time (s)")

lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, ncol=4, fontsize=7, loc="upper right")
ax.grid(True, alpha=0.3)

# Shared phase legend at bottom
carry_patch = mpatches.Patch(color="steelblue", alpha=0.7, label=f"Carry cmd  t={t_carry_cmd:.2f}s")
drop_patch  = mpatches.Patch(color="firebrick", alpha=0.7, label=f"Weld off   t={t_weld_off:.2f}s")
fig.legend(handles=[carry_patch, drop_patch], loc="lower center", ncol=2, fontsize=9,
           bbox_to_anchor=(0.5, 0.01))

plt.tight_layout(rect=[0, 0.04, 1, 1])
out_png = os.path.join(OUT_DIR, "sensor_plot.png")
plt.savefig(out_png, dpi=150)
print(f"Plot saved → {out_png}")
