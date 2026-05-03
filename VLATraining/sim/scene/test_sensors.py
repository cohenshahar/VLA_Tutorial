"""
Phase 7, Tasks 7.1–7.5 — Sensor suite verification.

Run from VLATraining/sim/:
    python -m scene.test_sensors

Sequence:
  1. Converge arm to snap pose, box pinned below EM pad.
  2. Zero velocities, activate weld.
  3. Run 500 settling steps.
  4. Read all 26 sensordata values, print with labels.
  5. Assert critical checks:
       a. All joint positions non-zero (arm not at home).
       b. Actuator torques non-zero (arm fighting gravity).
       c. EE force magnitude ≈ 4.9 N (0.5 kg box weight).
       d. Proximity sensor returns a float (even -1.0 = no hit is valid).

sensordata layout (26 floats):
  [0]      target_touch_sensor  (Phase 5, kept)
  [1:7]    sensor_pos_A1…A6     (joint positions, rad)
  [7:13]   sensor_vel_A1…A6     (joint velocities, rad/s)
  [13:19]  sensor_torque_A1…A6  (actuator forces, N·m)
  [19:22]  sensor_ee_force      (XYZ force at em_contact_site, site frame, N)
  [22:25]  sensor_ee_torque     (XYZ torque at em_contact_site, site frame, N·m)
  [25]     sensor_proximity     (rangefinder distance, m; -1.0 = no hit)
"""
import math, os, sys
import mujoco
import numpy as np

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
sys.path.insert(0, SIM_DIR)
from arm.load_arm import apply_gains
from scene.em_controller import em_activate

# ── Load ──────────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}
apply_gains(model, _GAINS)

# Snap pose: EM face pointing straight down (-Z), confirmed in Phase 6 fix
SNAP = {"a1": 0, "a2": -55, "a3": 90, "a4": 0, "a5": 55, "a6": 0}
ACT  = {f"a{i+1}": model.actuator(f"act_a{i+1}").id for i in range(6)}

em_site_id  = model.site("em_contact_site").id
box_bid     = model.body("metal_box").id
box_jnt_id  = model.joint("box_freejoint").id
box_qpa     = model.jnt_qposadr[box_jnt_id]
box_doa     = model.jnt_dofadr[box_jnt_id]
weld_id     = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")

# ── Step 1: find snap-pose EM contact-site position ────────────────────────
print("Step 1: finding snap-pose EM contact-site position …")
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)
for _ in range(3000):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
em_snap = data.site_xpos[em_site_id].copy()
print(f"  em_contact_site @ snap pose: {em_snap.round(4)}")

# ── Step 2: reset, place box 1.5 cm below EM site, set arm ctrl ────────────
print("\nStep 2: resetting scene, placing box under EM …")
mujoco.mj_resetData(model, data)
apply_gains(model, _GAINS)
BOX_HALF = 0.05
GAP      = 0.015
box_z0   = em_snap[2] - BOX_HALF - GAP
box_init = np.array([em_snap[0], em_snap[1], box_z0, 1., 0., 0., 0.])
data.qpos[box_qpa:box_qpa+7] = box_init
mujoco.mj_forward(model, data)
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)

# ── Step 3: converge arm (3000 steps, box pinned) ──────────────────────────
print("Step 3: converging arm to snap pose (3000 steps, box pinned) …")
for _ in range(3000):
    data.qpos[box_qpa:box_qpa+7] = box_init
    data.qvel[box_doa:box_doa+6] = 0.0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
# Zero all velocities for clean weld activation
data.qvel[:] = 0.0
data.qpos[box_qpa:box_qpa+7] = box_init
mujoco.mj_forward(model, data)

# ── Step 4: activate weld ─────────────────────────────────────────────────
print("Step 4: activating weld …")
ok = em_activate(model, data, threshold_m=0.05)
if not ok:
    print("  FAIL — em_activate returned False")
    raise SystemExit(1)
print(f"  Weld active: {bool(data.eq_active[weld_id])}")

# ── Step 5: 500 settling steps ─────────────────────────────────────────────
print("Step 5: 500 settling steps …")
for _ in range(500):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)

# ── Step 6: read & print all sensor values ─────────────────────────────────
sd = data.sensordata.copy()
print(f"\n{'─'*60}")
print(f"  SENSOR READOUT  (nsensordata={model.nsensordata})")
print(f"{'─'*60}")

# Helper: get sensordata slice by sensor name
def s(name):
    sid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    adr  = model.sensor_adr[sid]
    dim  = model.sensor_dim[sid]
    return sd[adr:adr+dim]

print(f"\n  [7.0] target_touch_sensor : {s('target_touch_sensor')[0]:+.4f}")

print(f"\n  [7.1] Joint positions (rad):")
for i in range(1, 7):
    v = s(f"sensor_pos_A{i}")[0]
    print(f"        sensor_pos_A{i}      : {v:+.4f} rad  ({math.degrees(v):+.2f}°)")

print(f"\n  [7.2] Joint velocities (rad/s):")
for i in range(1, 7):
    v = s(f"sensor_vel_A{i}")[0]
    print(f"        sensor_vel_A{i}      : {v:+.6f} rad/s")

print(f"\n  [7.3] Actuator torques (N·m):")
for i in range(1, 7):
    v = s(f"sensor_torque_A{i}")[0]
    print(f"        sensor_torque_A{i}   : {v:+.3f} N·m")

ee_force  = s("sensor_ee_force")
ee_torque = s("sensor_ee_torque")
ee_force_mag  = float(np.linalg.norm(ee_force))
ee_torque_mag = float(np.linalg.norm(ee_torque))
print(f"\n  [7.4] EE force  (site frame, N)  : {ee_force.round(4)}  |mag|={ee_force_mag:.3f} N")
print(f"        EE torque (site frame, N·m): {ee_torque.round(4)}  |mag|={ee_torque_mag:.3f} N·m")

prox = s("sensor_proximity")[0]
print(f"\n  [7.5] Proximity (rangefinder, m) : {prox:.4f}  (-1.0 = no hit)")

print(f"\n{'─'*60}")

# ── Step 7: assertions ─────────────────────────────────────────────────────
print("\nASSERTIONS:")
passes = 0

# 7.1 Joint positions: snap pose has A2, A3, A5 non-zero (A1=A4=A6=0 by design)
pos_vals = np.array([s(f"sensor_pos_A{i}")[0] for i in range(1, 7)])
nonzero  = np.sum(np.abs(pos_vals) > 0.01)
label    = "PASS ✓" if nonzero >= 3 else "FAIL ✗"
print(f"  7.1 Joint positions non-zero (≥3/6): {nonzero}/6  {label}")
if nonzero >= 3: passes += 1

# 7.2 Joint velocities: arm settled, all should be near-zero
vel_vals   = np.array([s(f"sensor_vel_A{i}")[0] for i in range(1, 7)])
max_vel    = float(np.max(np.abs(vel_vals)))
label      = "PASS ✓" if max_vel < 0.05 else "FAIL ✗"
print(f"  7.2 Joint velocities settled (max < 0.05 rad/s): {max_vel:.4f}  {label}")
if max_vel < 0.05: passes += 1

# 7.3 Actuator torques: at least some joints should be fighting gravity
trq_vals  = np.array([s(f"sensor_torque_A{i}")[0] for i in range(1, 7)])
max_trq   = float(np.max(np.abs(trq_vals)))
label     = "PASS ✓" if max_trq > 1.0 else "FAIL ✗"
print(f"  7.3 Max actuator torque > 1.0 N·m: {max_trq:.2f}  {label}")
if max_trq > 1.0: passes += 1

# 7.4 EE force ≈ box weight (0.5 kg × 9.81 = 4.905 N)
BOX_WEIGHT = 0.5 * 9.81
tol        = 1.5   # allow 1.5 N tolerance (constraint oscillation)
label      = "PASS ✓" if abs(ee_force_mag - BOX_WEIGHT) < tol else "FAIL ✗"
print(f"  7.4 EE force ≈ {BOX_WEIGHT:.3f} N  (got {ee_force_mag:.3f} N, tol ±{tol})  {label}")
if abs(ee_force_mag - BOX_WEIGHT) < tol: passes += 1

# 7.5 Proximity: must return a float (any value is valid output)
label = "PASS ✓" if np.isfinite(prox) else "FAIL ✗"
print(f"  7.5 Proximity is finite float: {prox:.4f}  {label}")
if np.isfinite(prox): passes += 1

print(f"\n{'═'*60}")
if passes == 5:
    print("  Phase 7 (7.1–7.5) ALL PASS ✓")
else:
    print(f"  Phase 7 (7.1–7.5) {passes}/5 passed  ✗")
print(f"{'═'*60}")
