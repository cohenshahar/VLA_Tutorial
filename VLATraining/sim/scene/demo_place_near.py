"""
scene/demo_place_near.py  —  Phase 10, Task 10.3
═══════════════════════════════════════════════════════════════════════════════
Full sequence: reach → grasp → lift → transport → place (gentle, 2 cm drop).

After lifting, the arm moves to above the target zone at "2 cm height":
  • EE (em_contact_site) descends until box bottom is 2 cm above target
    zone surface (target_z + 2 cm + 2×BOX_HALF = box_bottom + 2 cm).
  • EM deactivates → box falls 2 cm and lands on target zone.

Done when: target_touch_sensor > 0 within 1000 steps of EM deactivation.

Output
------
  outputs/demo_place_near_<YYYYMMDD>.mp4

Run:
  PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_place_near.py
"""

import math
import pathlib
import sys
from datetime import datetime

import imageio
import mujoco
import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate
from scene._ik_utils import R_DOWN, _run_ik, _sync_ctrl, _a1_toward

# ── I/O ───────────────────────────────────────────────────────────────────────
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
_DATE     = datetime.now().strftime("%Y%m%d")
OUT_VIDEO = str(_SIM_DIR / "outputs" / f"demo_place_near_{_DATE}.mp4")

# ── Scene geometry ────────────────────────────────────────────────────────────
TABLE_Z     = 0.85
BOX_HALF    = 0.05
BOX_POS     = np.array([0.76, 0.0, TABLE_Z + BOX_HALF])   # box centre at rest
TARGET_XY   = np.array([0.5, 0.4])                        # target zone XY
TARGET_Z    = 0.86                                         # target zone top surface
ARM_BASE    = np.array([0.0, -0.3])

# EE heights
APPROACH_Z  = TABLE_Z + 3.0 * (2 * BOX_HALF) + 0.05   # 1.20 m clearance
SNAP_Z      = BOX_POS[2] + BOX_HALF + 0.015            # hover = 0.965 m
LIFT_Z      = SNAP_Z + 0.15                            # lift 15 cm = 1.115 m

# "2 cm height" = box bottom is 2 cm above target zone surface when EM releases
# box_bottom = EE_Z - 2*BOX_HALF  →  EE_Z = TARGET_Z + 0.02 + 2*BOX_HALF
PLACE_NEAR_DROP = 0.02
PLACE_EE_Z = TARGET_Z + PLACE_NEAR_DROP + 2 * BOX_HALF   # = 0.98 m

# Done criterion
CONTACT_TIMEOUT = 1000   # steps after EM deactivation

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
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids   = [model.joint(n).id for n in _JOINT_NAMES]
    qpas   = [model.jnt_qposadr[j] for j in jids]
    dofas  = [model.jnt_dofadr[j]  for j in jids]
    ranges = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    site_id = model.site("em_contact_site").id

    # Touch sensor on target zone
    touch_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                   "target_touch_sensor")
    if touch_sid < 0:
        raise RuntimeError("target_touch_sensor not found in model.")
    touch_adr = model.sensor_adr[touch_sid]

    # Box freejoint
    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]
    box_doa = model.jnt_dofadr[box_jid]

    return model, data, qpas, dofas, ranges, act_ids, site_id, touch_adr, box_qpa, box_doa


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Run Task 10.3: reach → grasp → lift → transport → place near (2 cm)."""
    (model, data, qpas, dofas, ranges, act_ids,
     site_id, touch_adr, box_qpa, box_doa) = _load_model()

    # ── Video writer ──────────────────────────────────────────────────────────
    pathlib.Path(OUT_VIDEO).parent.mkdir(parents=True, exist_ok=True)
    fps    = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    writer = imageio.get_writer(OUT_VIDEO, fps=fps)
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

    # ── Reset ─────────────────────────────────────────────────────────────────
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)
    box_init = data.qpos[box_qpa:box_qpa + 7].copy()
    _sync_ctrl(data, qpas, act_ids)
    mujoco.mj_forward(model, data)

    # ── 1. Home settle ────────────────────────────────────────────────────────
    print("[1] Settling at home …")
    _run_n(1000)

    # ── 2. Pre-align A1 toward box ────────────────────────────────────────────
    a1_tgt = _a1_toward(BOX_POS, ARM_BASE, ranges)
    print(f"[2] Pre-align A1 → {math.degrees(a1_tgt):.1f}° …")
    data.qpos[qpas[0]]    = a1_tgt
    data.ctrl[act_ids[0]] = a1_tgt
    _run_n(2000)

    # ── 3. Rise to clearance ──────────────────────────────────────────────────
    ee_xy = data.site_xpos[site_id][:2].copy()
    print(f"[3] IK: rise to Z = {APPROACH_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000)

    # ── 4. Translate over box ─────────────────────────────────────────────────
    print(f"[4] IK: translate over box …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], APPROACH_Z]),
                      R_DOWN, max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)

    # ── 5. Descend to snap height (box pinned) ────────────────────────────────
    print(f"[5] IK: descend to snap Z = {SNAP_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], SNAP_Z]),
                      R_DOWN, pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    for _ in range(1500):
        data.qpos[box_qpa:box_qpa + 7] = box_init
        data.qvel[box_doa:box_doa + 6] = 0.0
        _physics_step()

    # ── 6. EM activate ────────────────────────────────────────────────────────
    data.qvel[:] = 0.0
    data.qpos[box_qpa:box_qpa + 7] = box_init
    mujoco.mj_forward(model, data)
    activated = em_activate(model, data, threshold_m=0.05)
    print(f"[6] EM activate: {activated}")
    if not activated:
        print("    WARNING: EM did not activate.")
    _run_n(300)

    # ── 7. Lift ───────────────────────────────────────────────────────────────
    ee_xy_now = data.site_xpos[site_id][:2].copy()
    print(f"[7] IK: lift to Z = {LIFT_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy_now[0], ee_xy_now[1], LIFT_Z]))
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)

    # ── 8. Pre-align A1 toward target zone ────────────────────────────────────
    a1_target = _a1_toward(TARGET_XY, ARM_BASE, ranges)
    print(f"[8] Pre-align A1 → {math.degrees(a1_target):.1f}° toward target zone …")
    data.qpos[qpas[0]]    = a1_target
    data.ctrl[act_ids[0]] = a1_target
    _run_n(2000)

    # ── 9. Translate over target zone ─────────────────────────────────────────
    print(f"[9] IK: translate over target zone ({TARGET_XY[0]:.2f}, {TARGET_XY[1]:.2f}) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([TARGET_XY[0], TARGET_XY[1], LIFT_Z]),
                      max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)

    # ── 10. Descend to place height (2 cm above target zone surface) ──────────
    print(f"[10] IK: descend to place Z = {PLACE_EE_Z:.3f} m "
          f"(box bottom {PLACE_NEAR_DROP*100:.0f} cm above target) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([TARGET_XY[0], TARGET_XY[1], PLACE_EE_Z]),
                      pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000)

    # ── 11. EM deactivate → box falls 2 cm ───────────────────────────────────
    print("[11] EM deactivate — box drops …")
    em_deactivate(model, data)

    # ── 12. Wait for contact (timeout = 1000 steps) ───────────────────────────
    print(f"[12] Waiting for target_contact (timeout = {CONTACT_TIMEOUT} steps) …")
    done = False
    for i in range(CONTACT_TIMEOUT):
        _physics_step()
        touch = float(data.sensordata[touch_adr])
        if touch > 0.0:
            done = True
            print(f"    ✅ DONE at step {i}: target_contact = True")
            _run_n(500)
            break

    if not done:
        print(f"    ❌ No contact within {CONTACT_TIMEOUT} steps.")

    # ── Finalise ──────────────────────────────────────────────────────────────
    writer.close()
    print(f"\nVideo : {OUT_VIDEO} ({step_n[0] // RENDER_SKIP} frames @ {fps} fps)")


if __name__ == "__main__":
    main()
