"""
scene/demo_full_sequence.py  —  Phase 10, Task 10.5
═══════════════════════════════════════════════════════════════════════════════
Combines Tasks 10.1–10.3 into one script:
  reach → grasp → lift → transport → place near

Video: wrist camera (left) + overhead camera (right) side-by-side, 1280×720.
Text overlay (top-left of frame):
  • Current step name
  • Proximity (m)
  • EE Fz (N)
  • EM state (ON / OFF)
  • Target contact (YES / NO)

Text rendering uses OpenCV (cv2) if available, otherwise falls back to
plain numpy pixel stamping (legibility not guaranteed without cv2).

Output
------
  outputs/demo_full_sequence_<YYYYMMDD>.mp4

Run:
  PYTHONPATH=VLATraining/sim python3 VLATraining/sim/scene/demo_full_sequence.py
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

# ── Optional cv2 for text overlay ─────────────────────────────────────────────
try:
    import cv2 as _cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False
    print("WARNING: cv2 not found — text overlay disabled. Install opencv-python.")

# ── I/O ───────────────────────────────────────────────────────────────────────
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
_DATE     = datetime.now().strftime("%Y%m%d")
OUT_VIDEO = str(_SIM_DIR / "outputs" / f"demo_full_sequence_{_DATE}.mp4")

# ── Scene geometry ────────────────────────────────────────────────────────────
TABLE_Z    = 0.85
BOX_HALF   = 0.05
BOX_POS    = np.array([0.76, 0.0, TABLE_Z + BOX_HALF])
TARGET_XY  = np.array([0.5, 0.4])
TARGET_Z   = 0.86
ARM_BASE   = np.array([0.0, -0.3])

APPROACH_Z = TABLE_Z + 3.0 * (2 * BOX_HALF) + 0.05   # 1.20 m
SNAP_Z     = BOX_POS[2] + BOX_HALF + 0.015            # 0.965 m
LIFT_Z     = SNAP_Z + 0.15                            # 1.115 m
PLACE_EE_Z = TARGET_Z + 0.02 + 2 * BOX_HALF           # 0.98 m  (2 cm drop)

CONTACT_TIMEOUT = 1000

# ── Video layout ──────────────────────────────────────────────────────────────
# Each camera panel is half the final width, full height.
PANEL_W, PANEL_H = 640, 720          # per-camera panel dimensions
VID_W = PANEL_W * 2                  # 1280 total
VID_H = PANEL_H                      # 720 total
RENDER_SKIP = 16

# Camera names (must match world.xml)
CAM_WRIST    = "cam_wrist"
CAM_OVERHEAD = "cam_overhead"

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

    def _sensor_adr(name):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sid < 0:
            raise RuntimeError(f"Sensor '{name}' not found in model.")
        return model.sensor_adr[sid]

    prox_adr  = _sensor_adr("sensor_proximity")
    ft_adr    = _sensor_adr("sensor_ee_force")   # fz at ft_adr + 2
    touch_adr = _sensor_adr("target_touch_sensor")

    weld_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]
    box_doa = model.jnt_dofadr[box_jid]

    return (model, data, qpas, dofas, ranges, act_ids, site_id,
            prox_adr, ft_adr, touch_adr, weld_id, box_qpa, box_doa)


# ══════════════════════════════════════════════════════════════════════════════
#  Text overlay helpers
# ══════════════════════════════════════════════════════════════════════════════

def _overlay_text(frame_rgb, lines):
    """
    Add multi-line text to an RGB numpy array.
    Uses cv2 if available; otherwise returns frame unchanged.

    Parameters
    ----------
    frame_rgb : (H, W, 3) uint8 RGB array.
    lines     : List of strings to render, top-to-bottom.

    Returns
    -------
    (H, W, 3) uint8 RGB array with text drawn.
    """
    if not _HAS_CV2:
        return frame_rgb
    # cv2 works in BGR
    frame_bgr = _cv2.cvtColor(frame_rgb, _cv2.COLOR_RGB2BGR)
    font       = _cv2.FONT_HERSHEY_SIMPLEX
    scale      = 0.65
    thickness  = 2
    color_text = (255, 255,  64)    # yellow
    color_bg   = (  0,   0,   0)   # black shadow

    for i, line in enumerate(lines):
        x, y = 10, 28 + i * 28
        # Drop shadow
        _cv2.putText(frame_bgr, line, (x + 1, y + 1),
                     font, scale, color_bg, thickness, _cv2.LINE_AA)
        _cv2.putText(frame_bgr, line, (x, y),
                     font, scale, color_text, thickness, _cv2.LINE_AA)
    return _cv2.cvtColor(frame_bgr, _cv2.COLOR_BGR2RGB)


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Run Task 10.5: full reach→grasp→lift→transport→place sequence."""
    (model, data, qpas, dofas, ranges, act_ids, site_id,
     prox_adr, ft_adr, touch_adr, weld_id,
     box_qpa, box_doa) = _load_model()

    # ── Video writer (streaming) ──────────────────────────────────────────────
    pathlib.Path(OUT_VIDEO).parent.mkdir(parents=True, exist_ok=True)
    fps    = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    writer = imageio.get_writer(OUT_VIDEO, fps=fps)

    # Two renderers: wrist (left) + overhead (right)
    ren_wrist    = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)
    ren_overhead = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)

    step_n      = [0]
    _step_name  = ["init"]

    def _read_sensors():
        sd      = data.sensordata
        prox    = float(sd[prox_adr])
        ee_fz   = float(sd[ft_adr + 2])
        touch   = float(sd[touch_adr]) > 0.0
        em_on   = bool(data.eq_active[weld_id])
        return prox, ee_fz, em_on, touch

    def _physics_step():
        mujoco.mj_step(model, data)
        if step_n[0] % RENDER_SKIP == 0:
            prox, ee_fz, em_on, touch = _read_sensors()
            lines = [
                f"Phase: {_step_name[0]}",
                f"Prox: {prox:.4f} m",
                f"EE Fz: {ee_fz:+.2f} N",
                f"EM: {'ON' if em_on else 'OFF'}",
                f"Contact: {'YES' if touch else 'NO'}",
            ]
            ren_wrist.update_scene(data, camera=CAM_WRIST)
            left = ren_wrist.render().copy()

            ren_overhead.update_scene(data, camera=CAM_OVERHEAD)
            right = ren_overhead.render().copy()

            # Combine side-by-side, then add text overlay on left panel
            left  = _overlay_text(left,  lines)
            frame = np.concatenate([left, right], axis=1)
            writer.append_data(frame)
        step_n[0] += 1

    def _run_n(n, name=None):
        if name:
            _step_name[0] = name
        for _ in range(n):
            _physics_step()

    # ── Reset ─────────────────────────────────────────────────────────────────
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)
    box_init = data.qpos[box_qpa:box_qpa + 7].copy()
    _sync_ctrl(data, qpas, act_ids)
    mujoco.mj_forward(model, data)

    # ── 1. Home settle ────────────────────────────────────────────────────────
    print("[1] Home settle …")
    _run_n(1000, "home")

    # ── 2. Pre-align A1 ───────────────────────────────────────────────────────
    a1_tgt = _a1_toward(BOX_POS, ARM_BASE, ranges)
    print(f"[2] Pre-align A1 → {math.degrees(a1_tgt):.1f}° …")
    data.qpos[qpas[0]]    = a1_tgt
    data.ctrl[act_ids[0]] = a1_tgt
    _run_n(2000, "A1 align")

    # ── 3. Rise to clearance ──────────────────────────────────────────────────
    ee_xy = data.site_xpos[site_id][:2].copy()
    print(f"[3] IK: rise to Z = {APPROACH_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000, "rise")

    # ── 4. Translate over box ─────────────────────────────────────────────────
    print("[4] IK: translate over box …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], APPROACH_Z]),
                      R_DOWN, max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500, "reach")

    # ── 5. Descend to snap height (box pinned) ────────────────────────────────
    print(f"[5] IK: descend to snap Z = {SNAP_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([BOX_POS[0], BOX_POS[1], SNAP_Z]),
                      R_DOWN, pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _step_name[0] = "descend"
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
    _run_n(300, "grasp")

    # ── 7. Lift ───────────────────────────────────────────────────────────────
    ee_xy_now = data.site_xpos[site_id][:2].copy()
    print(f"[7] IK: lift to Z = {LIFT_Z:.3f} m …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy_now[0], ee_xy_now[1], LIFT_Z]))
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500, "lift")

    # ── 8. Pre-align A1 toward target ─────────────────────────────────────────
    a1_target = _a1_toward(TARGET_XY, ARM_BASE, ranges)
    print(f"[8] Pre-align A1 → {math.degrees(a1_target):.1f}° toward target …")
    data.qpos[qpas[0]]    = a1_target
    data.ctrl[act_ids[0]] = a1_target
    _run_n(2000, "transport")

    # ── 9. Translate over target ───────────────────────────────────────────────
    print(f"[9] IK: translate over target ({TARGET_XY[0]:.2f}, {TARGET_XY[1]:.2f}) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([TARGET_XY[0], TARGET_XY[1], LIFT_Z]),
                      max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500, "over target")

    # ── 10. Descend to place height ───────────────────────────────────────────
    print(f"[10] IK: descend to place Z = {PLACE_EE_Z:.3f} m (2 cm gentle drop) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([TARGET_XY[0], TARGET_XY[1], PLACE_EE_Z]),
                      pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000, "place descend")

    # ── 11. EM deactivate → box drops 2 cm ───────────────────────────────────
    print("[11] EM deactivate — box drops 2 cm …")
    em_deactivate(model, data)

    # ── 12. Wait for contact ──────────────────────────────────────────────────
    print(f"[12] Waiting for contact (timeout = {CONTACT_TIMEOUT} steps) …")
    done = False
    for i in range(CONTACT_TIMEOUT):
        _run_n(1, "place")
        touch = float(data.sensordata[touch_adr]) > 0.0
        if touch:
            done = True
            print(f"    ✅ DONE at step {i}: target_contact = True")
            _run_n(800, "done")
            break

    if not done:
        print(f"    ❌ No contact within {CONTACT_TIMEOUT} steps.")
        _run_n(300, "timeout")

    # ── Finalise ──────────────────────────────────────────────────────────────
    writer.close()
    total_frames = step_n[0] // RENDER_SKIP
    print(f"\nVideo : {OUT_VIDEO}  ({total_frames} frames @ {fps} fps)")
    print(f"cv2   : {'enabled' if _HAS_CV2 else 'disabled (no text overlay)'}")


if __name__ == "__main__":
    main()
