"""
Phase 6, Task 6.2 — Activate the weld constraint manually and observe.

Run from VLATraining/sim/:
    python -m scene.test_task_6_2

Sequence:
  1. Find snap-pose EM contact-site position (simulate arm, record site, reset).
  2. Place box 3 cm below EM site. Set arm ctrl → snap pose.
  3. Run 3000 steps while PINNING the box (same approach as em_pickup_test.py).
     Arm converges to snap pose; box stays 3 cm below EM within threshold.
  4. Call em_activate() — computes correct relpose, activates weld.
  5. Release pin. Run 2000 more steps printing box Z every 200 steps.

Pass condition: box Z drifts < 10 cm from where it was at activation.
"""
import math, os, sys
import mujoco
import numpy as np

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")
sys.path.insert(0, SIM_DIR)
from scene.em_controller import em_activate, em_can_activate

# ── Load ──────────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}
for name, (kp, kv) in _GAINS.items():
    ai = model.actuator(name).id
    model.actuator_gainprm[ai, 0] =  kp
    model.actuator_biasprm[ai, 1] = -kp
    model.actuator_biasprm[ai, 2] = -kv
for i in range(model.nv):
    model.dof_armature[i] = 0.5

ACT  = {f"a{i+1}": model.actuator(f"act_a{i+1}").id for i in range(6)}
SNAP = {"a1": 0, "a2": -60, "a3": 50, "a4": 0, "a5": 30, "a6": 0}

em_site_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,  "em_contact_site")
box_bid     = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,  "metal_box")
box_jnt_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_freejoint")
box_qpa     = model.jnt_qposadr[box_jnt_id]
box_doa     = model.jnt_dofadr[box_jnt_id]
weld_id     = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")

# ── Step 1: find EM position in snap pose ─────────────────────────────────
print("Step 1: finding snap-pose EM position …")
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)
for _ in range(3000):
    mujoco.mj_step(model, data)
mujoco.mj_forward(model, data)
em_snap = data.site_xpos[em_site_id].copy()
print(f"  em_contact_site @ snap pose: {em_snap.round(4)}")

# ── Step 2: reset + place box 3 cm below EM site ─────────────────────────
print("\nStep 2: resetting, placing box under EM, setting arm to snap pose …")
mujoco.mj_resetData(model, data)
BOX_HALF = 0.05
GAP      = 0.03                               # box top → EM contact gap
box_z0   = em_snap[2] - BOX_HALF - GAP        # box centre Z
box_init = np.array([em_snap[0], em_snap[1], box_z0, 1., 0., 0., 0.])
data.qpos[box_qpa:box_qpa+7] = box_init
mujoco.mj_forward(model, data)
for name, deg in SNAP.items():
    data.ctrl[ACT[name]] = math.radians(deg)
print(f"  Box centre: {data.xpos[box_bid].round(4)}")
print(f"  box_top → EM gap: {(em_snap[2] - (box_z0 + BOX_HALF))*100:.1f} cm  (threshold 5 cm)")

# ── Step 3: run arm to snap pose while pinning box ────────────────────────
print("\nStep 3: converging arm to snap pose, box pinned (3000 steps) …")
for _ in range(3000):
    data.qpos[box_qpa:box_qpa+7] = box_init
    data.qvel[box_doa:box_doa+6] = 0.0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)

can = em_can_activate(data, model, threshold_m=0.05)
print(f"  em_can_activate after convergence: {can}  (expected True)")
if not can:
    box_top_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "box_top_site")
    d = float(np.linalg.norm(data.site_xpos[em_site_id] - data.site_xpos[box_top_id]))
    print(f"  Site distance = {d*100:.1f} cm — arm not close enough")
    raise SystemExit(1)

# ── Step 4: activate weld ─────────────────────────────────────────────────
print("\nStep 4: activating weld via em_activate() …")
data.qpos[box_qpa:box_qpa+7] = box_init
data.qvel[box_doa:box_doa+6] = 0.0
mujoco.mj_forward(model, data)

ok = em_activate(model, data, threshold_m=0.05)
print(f"  em_activate returned: {ok}")
print(f"  data.eq_active[weld] = {data.eq_active[weld_id]}")
print(f"  eq_data relpos  = {model.eq_data[weld_id, 0:3].round(4)}")
print(f"  eq_data relquat = {model.eq_data[weld_id, 6:10].round(4)}")
box_z_at_activation = float(data.xpos[box_bid][2])
print(f"  Box Z at activation: {box_z_at_activation:.4f}")

if not ok:
    print("  FAIL — weld did not activate")
    raise SystemExit(1)

# ── Step 5: run 500 settling steps (arm + box find equilibrium) ──────────
print(f"\nStep 5a: 500 settling steps after weld activation …")
for _ in range(500):
    mujoco.mj_step(model, data)
box_z_settled = float(data.xpos[box_bid][2])
print(f"  Box Z after settling: {box_z_settled:.4f}  (above table at 0.90 m: {box_z_settled > 1.1})")

# ── Step 5b: 2000 measurement steps — measure stability ──────────────────
print(f"\nStep 5b: 2000 measurement steps …")
print(f"  {'Step':>6}  {'Box X':>8}  {'Box Y':>8}  {'Box Z':>8}")
z_vals = []
for step in range(1, 2001):
    mujoco.mj_step(model, data)
    z_vals.append(float(data.xpos[box_bid][2]))
    if step % 200 == 0:
        p = data.xpos[box_bid]
        print(f"  {step:>6}  {p[0]:>8.4f}  {p[1]:>8.4f}  {p[2]:>8.4f}")

z_drift = max(z_vals) - min(z_vals)
box_z_final = float(data.xpos[box_bid][2])
print(f"\n  Box Z at activation : {box_z_at_activation:.4f}")
print(f"  Box Z after settle  : {box_z_settled:.4f}")
print(f"  Z peak-to-peak drift: {z_drift*100:.2f} cm  (pass threshold: 2 cm)")
print(f"  Box well above table: {box_z_settled > 1.1}  (settled Z = {box_z_settled:.4f} > 1.1)")

if z_drift < 0.02 and box_z_settled > 1.1:
    print("\nTask 6.2 PASS ✓  — weld holds box steady above table (p-p drift < 2 cm)")
else:
    print("\nTask 6.2 FAIL ✗  — box drifted too much or not held above table")
