"""
scene/demo_reach.py  —  Phase 10, Task 10.1
═══════════════════════════════════════════════════════════════════════════════
Arm moves from home pose (all joints = 0 rad) to hover above the metal box,
EM face pointing straight down.

Done when: proximity sensor reads < 0.025 m.
Prints proximity at every physics step during the final descent.

Output
------
  outputs/demo_reach_<YYYYMMDD>.mp4   ← streaming video (no frame list in RAM)
  sensor log printed to stdout

Run (from repo root):
  PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_reach.py
"""

import math
import pathlib
import sys
from datetime import datetime

import imageio
import mujoco
import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent   # VLATraining/sim/
sys.path.insert(0, str(_SIM_DIR))

from arm.load_arm import apply_gains
from scene._ik_utils import R_DOWN, _run_ik, _sync_ctrl, _a1_toward

# ── I/O paths ─────────────────────────────────────────────────────────────────
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
_DATE     = datetime.now().strftime("%Y%m%d")
OUT_VIDEO = str(_SIM_DIR / "outputs" / f"demo_reach_{_DATE}.mp4")

# ── Scene geometry (fixed — no randomisation) ─────────────────────────────────
TABLE_Z  = 0.85                                     # tabletop surface (m)
BOX_HALF = 0.05                                     # box half-size (m)
BOX_POS  = np.array([0.76, 0.0, TABLE_Z + BOX_HALF])  # box centre (world)
ARM_BASE = np.array([0.0, -0.3])                   # arm XY base in world

APPROACH_Z = TABLE_Z + 3.0 * (2 * BOX_HALF) + 0.05   # clearance = 1.20 m
SNAP_Z     = BOX_POS[2] + BOX_HALF + 0.015            # hover = box_top + 1.5 cm = 0.965 m

PROX_DONE = 0.025   # success criterion (m)

# ── Render / video ────────────────────────────────────────────────────────────
RENDER_SKIP  = 16
VID_W, VID_H = 1280, 720

# ── PD gains ──────────────────────────────────────────────────────────────────
_GAINS = {
    "act_a1": (800., 80.),
    "act_a2": (800., 80.),
    "act_a3": (500., 50.),
    "act_a4": (300., 30.),
    "act_a5": (500., 50.),
    "act_a6": (300., 30.),
}

_JOINT_NAMES = [f"joint_a{i}" for i in range(1, 7)]
_ACT_NAMES   = [f"act_a{i}"   for i in range(1, 7)]


# ══════════════════════════════════════════════════════════════════════════════
#  Model loading
# ══════════════════════════════════════════════════════════════════════════════

def _load_model():
    """Load MuJoCo model; resolve all joint/sensor addresses up front."""
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids   = [model.joint(n).id for n in _JOINT_NAMES]
    qpas   = [model.jnt_qposadr[j] for j in jids]
    dofas  = [model.jnt_dofadr[j]  for j in jids]
    ranges = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    site_id = model.site("em_contact_site").id

    prox_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "sensor_proximity")
    if prox_sid < 0:
        raise RuntimeError("sensor_proximity not found in model.")
    prox_adr = model.sensor_adr[prox_sid]

    return model, data, qpas, dofas, ranges, act_ids, site_id, prox_adr


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Run Task 10.1: home → hover above metal box."""
    model, data, qpas, dofas, ranges, act_ids, site_id, prox_adr = _load_model()

    # ── Video writer (streaming — O(1) memory) ────────────────────────────────
    pathlib.Path(OUT_VIDEO).parent.mkdir(parents=True, exist_ok=True)
    fps    = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    writer = imageio.get_writer(OUT_VIDEO, fps=fps)

    # ── Renderer ──────────────────────────────────────────────────────────────
    renderer = mujoco.Renderer(model, height=VID_H, width=VID_W)
    step_n   = [0]

    def _physics_step():
        mujoco.mj_step(model, data)
        if step_n[0] % RENDER_SKIP == 0:
            renderer.update_scene(data)
            writer.append_data(renderer.render())
        step_n[0] += 1

    def _run_n(n):
        for _ in range(n):
            _physics_step()

    # ── Reset to home pose (all joints = 0) ───────────────────────────────────
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)
    _sync_ctrl(data, qpas, act_ids)   # hold home with PD
    mujoco.mj_forward(model, data)
    ee0 = data.site_xpos[site_id].copy()
    print(f"[init] Home pose — EE: ({ee0[0]:.3f}, {ee0[1]:.3f}, {ee0[2]:.3f})")

    # ── 1. Settle at home ─────────────────────────────────────────────────────
    print("[1] Settling at home pose (1 s) …")
    _run_n(1000)

    # ── 2. Pre-align A1 toward box (analytic) ─────────────────────────────────
    a1_tgt = _a1_toward(BOX_POS, ARM_BASE, ranges)
    print(f"[2] Pre-align A1 → {math.degrees(a1_tgt):.1f}° toward box …")
    data.qpos[qpas[0]]    = a1_tgt
    data.ctrl[act_ids[0]] = a1_tgt
    _run_n(2000)   # 2 s to swing A1

    # ── 3. Rise to clearance height ───────────────────────────────────────────
    ee_xy = data.site_xpos[site_id][:2].copy()
    print(f"[3] IK: rise to clearance Z = {APPROACH_Z:.3f} m, EM pointing down …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000)

    # ── 4. Translate over box XY ──────────────────────────────────────────────
    print(f"[4] IK: translate to ({BOX_POS[0]:.3f}, {BOX_POS[1]:.3f}, {APPROACH_Z:.3f}) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], APPROACH_Z]),
                      R_DOWN, max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)

    # ── 5. Descend to hover height ────────────────────────────────────────────
    print(f"[5] IK: descend to hover Z = {SNAP_Z:.3f} m (box_top + 1.5 cm) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], SNAP_Z]),
                      R_DOWN, pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)

    # ── 6. Physics settle — print proximity every step ────────────────────────
    print(f"\n[6] Proximity log (done when < {PROX_DONE} m):")
    print(f"    {'step':>6}  proximity (m)")
    print(f"    {'-'*25}")
    done     = False
    done_step = -1
    for i in range(3000):
        _physics_step()
        prox = float(data.sensordata[prox_adr])
        print(f"    {i:6d}  {prox:.4f}")
        if prox < PROX_DONE and not done:
            done      = True
            done_step = i
            print(f"\n    ✅ DONE at step {i}: proximity = {prox:.4f} m < {PROX_DONE} m")
            _run_n(300)   # hold position for 0.3 s, then exit
            break

    if not done:
        print(f"\n    ❌ Did not reach proximity < {PROX_DONE} m in 3000 steps.")
        prox = float(data.sensordata[prox_adr])
        print(f"    Final proximity = {prox:.4f} m")

    # ── Finalise ──────────────────────────────────────────────────────────────
    writer.close()
    ee_final = data.site_xpos[site_id].copy()
    print(f"\nFinal EE  : ({ee_final[0]:.3f}, {ee_final[1]:.3f}, {ee_final[2]:.3f})")
    print(f"Done step : {done_step}")
    print(f"Video     : {OUT_VIDEO} ({step_n[0] // RENDER_SKIP} frames @ {fps} fps)")


if __name__ == "__main__":
    main()
