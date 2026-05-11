"""
scene/demo_grasp_lift.py  —  Phase 10, Task 10.2
═══════════════════════════════════════════════════════════════════════════════
Arm reaches the metal box, EM activates, then lifts 15 cm.

Done when: FT sensor reads |ee_fz| > 4.0 N for 100 consecutive steps after
the lift command is issued.  ee_fz is the 3rd component of sensor_ee_force
(site-local Z at em_contact_site).

With --object sphere  the script targets metal_sphere instead (Task 10.6).
Default: metal_box.

Output
------
  outputs/demo_grasp_lift_<YYYYMMDD>.mp4

Run:
  PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_grasp_lift.py
  PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_grasp_lift.py --object sphere
"""

import argparse
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

# ── Scene geometry ────────────────────────────────────────────────────────────
TABLE_Z  = 0.85
BOX_HALF = 0.05
ARM_BASE = np.array([0.0, -0.3])

# Object configs: centre position when resting on table
_OBJECTS = {
    "box":    {"pos": np.array([0.76,  0.0, TABLE_Z + BOX_HALF]),
               "half": BOX_HALF,
               "freejoint": "box_freejoint"},
    "sphere": {"pos": np.array([0.55,  0.15, TABLE_Z + 0.04]),
               "half": 0.04,
               "freejoint": "sphere_freejoint"},
}

APPROACH_Z  = TABLE_Z + 3.0 * (2 * BOX_HALF) + 0.05   # 1.20 m clearance
LIFT_HEIGHT = 0.15   # lift 15 cm above snap position

# FT done criterion
FT_THRESH      = 4.0   # N
FT_CONSEC_NEED = 100   # consecutive steps

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

    # FT sensor: sensor_ee_force → [fx, fy, fz] in site local frame
    ft_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "sensor_ee_force")
    if ft_sid < 0:
        raise RuntimeError("sensor_ee_force not found in model.")
    ft_adr = model.sensor_adr[ft_sid]   # fz is at ft_adr + 2

    return model, data, qpas, dofas, ranges, act_ids, site_id, ft_adr


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Run Task 10.2: reach → EM activate → lift 15 cm."""
    parser = argparse.ArgumentParser(
        description="Phase 10, Task 10.2 — grasp and lift.")
    parser.add_argument("--object", choices=["box", "sphere"], default="box",
                        help="Object to grasp (default: box)")
    args = parser.parse_args()

    obj_cfg  = _OBJECTS[args.object]
    obj_pos  = obj_cfg["pos"]         # object centre in world
    obj_half = obj_cfg["half"]        # half-height (for snap Z calculation)
    snap_gap = 0.015
    snap_z   = obj_pos[2] + obj_half + snap_gap   # EE hover above object top
    lift_z   = snap_z + LIFT_HEIGHT               # EE after 15 cm lift

    out_video = str(_SIM_DIR / "outputs" / f"demo_grasp_lift_{_DATE}.mp4")
    print(f"Object   : {args.object}")
    print(f"Snap Z   : {snap_z:.3f} m")
    print(f"Lift Z   : {lift_z:.3f} m")

    model, data, qpas, dofas, ranges, act_ids, site_id, ft_adr = _load_model()

    # ── Video writer ──────────────────────────────────────────────────────────
    pathlib.Path(out_video).parent.mkdir(parents=True, exist_ok=True)
    fps    = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    writer = imageio.get_writer(out_video, fps=fps)
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
    _sync_ctrl(data, qpas, act_ids)
    mujoco.mj_forward(model, data)

    # ── 1. Home settle ────────────────────────────────────────────────────────
    print("[1] Settling at home …")
    _run_n(1000)

    # ── 2. Pre-align A1 ───────────────────────────────────────────────────────
    a1_tgt = _a1_toward(obj_pos, ARM_BASE, ranges)
    print(f"[2] Pre-align A1 → {math.degrees(a1_tgt):.1f}° …")
    data.qpos[qpas[0]]    = a1_tgt
    data.ctrl[act_ids[0]] = a1_tgt
    _run_n(2000)

    # ── 3. Rise to clearance ──────────────────────────────────────────────────
    ee_xy = data.site_xpos[site_id][:2].copy()
    print(f"[3] IK: rise to clearance Z = {APPROACH_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000)

    # ── 4. Translate over object ──────────────────────────────────────────────
    print(f"[4] IK: translate over {args.object} ({obj_pos[0]:.3f}, {obj_pos[1]:.3f}) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([obj_pos[0], obj_pos[1], APPROACH_Z]),
                      R_DOWN, max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)

    # ── 5. Descend to snap height (object pinned) ─────────────────────────────
    # Resolve object freejoint for pinning
    obj_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, obj_cfg["freejoint"])
    obj_qpa  = model.jnt_qposadr[obj_jid]
    obj_doa  = model.jnt_dofadr[obj_jid]
    obj_init = data.qpos[obj_qpa:obj_qpa + 7].copy()

    print(f"[5] IK: descend to snap Z = {snap_z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([obj_pos[0], obj_pos[1], snap_z]),
                      R_DOWN, pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)

    # Settle with object pinned (prevent drift before grasp)
    for _ in range(1500):
        data.qpos[obj_qpa:obj_qpa + 7] = obj_init
        data.qvel[obj_doa:obj_doa + 6] = 0.0
        _physics_step()

    # ── 6. EM activate ────────────────────────────────────────────────────────
    data.qvel[:] = 0.0
    data.qpos[obj_qpa:obj_qpa + 7] = obj_init
    mujoco.mj_forward(model, data)
    activated = em_activate(model, data, threshold_m=0.05)
    print(f"[6] EM activate: {activated}")
    if not activated:
        print("    WARNING: EM did not activate. Check snap_z / threshold.")
    _run_n(300)

    # ── 7. IK: lift 15 cm ─────────────────────────────────────────────────────
    ee_xy_now = data.site_xpos[site_id][:2].copy()
    print(f"[7] IK: lift to Z = {lift_z:.3f} m (+{LIFT_HEIGHT:.2f} m) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy_now[0], ee_xy_now[1], lift_z]))
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)

    # ── 8. Physics settle + FT monitoring ─────────────────────────────────────
    print(f"\n[8] Lift physics — monitoring FT (ee_fz, threshold {FT_THRESH} N "
          f"for {FT_CONSEC_NEED} consecutive steps):")
    consec  = 0
    done    = False
    for i in range(3000):
        _physics_step()
        fz = float(data.sensordata[ft_adr + 2])
        if abs(fz) > FT_THRESH:
            consec += 1
        else:
            consec = 0
        if i % 100 == 0:
            print(f"    step {i:5d}  ee_fz = {fz:+.3f} N  consec = {consec}")
        if consec >= FT_CONSEC_NEED:
            done = True
            print(f"\n    ✅ DONE at step {i}: |ee_fz| = {abs(fz):.3f} N "
                  f"exceeded {FT_THRESH} N for {FT_CONSEC_NEED} consecutive steps.")
            _run_n(500)
            break

    if not done:
        fz = float(data.sensordata[ft_adr + 2])
        print(f"\n    ❌ Did not reach FT criterion.  Final ee_fz = {fz:+.3f} N, "
              f"consec = {consec}")

    # ── Finalise ──────────────────────────────────────────────────────────────
    writer.close()
    print(f"\nVideo : {out_video} ({step_n[0] // RENDER_SKIP} frames @ {fps} fps)")


if __name__ == "__main__":
    main()
