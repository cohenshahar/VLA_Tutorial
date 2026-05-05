"""
scene/record_multicam.py  —  record one pick-carry-drop cycle from 3 cameras side-by-side
Saves:  outputs/multicam_cycle.mp4   (cam_overhead | cam_side | cam_wrist)
Usage:  PYTHONPATH=. python scene/record_multicam.py

Rendering backend:
  By default sets MUJOCO_GL=egl (GPU offscreen).
  If you have no GPU, override before running:
    MUJOCO_GL=osmesa python scene/record_multicam.py
"""

import os
# Set offscreen rendering backend before importing mujoco.
# 'egl'    — GPU-backed (fast, requires GPU or EGL libs)
# 'osmesa' — software (slow but works everywhere, needs libosmesa6)
# Without this, MuJoCo may silently return black frames.
os.environ.setdefault("MUJOCO_GL", "egl")

import math
import pathlib
import sys
import time

import imageio
import mujoco
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate

# ── paths ─────────────────────────────────────────────────────────────────────
WORLD_XML  = str(pathlib.Path(__file__).parent / "world.xml")
OUTPUT_MP4 = str(pathlib.Path(__file__).parent.parent / "outputs" / "multicam_cycle.mp4")

# ── video settings ────────────────────────────────────────────────────────────
CAM_W, CAM_H = 480, 320       # per-camera resolution
VIDEO_FPS    = 30
CAMS = ["cam_overhead", "cam_side", "cam_wrist"]

# ── simulation constants (match view_pickup_throw.py) ─────────────────────────
TABLE_Z    = 0.85
BOX_HALF   = 0.05
ARM_BASE   = np.array([0.0, -0.3])
EXCL_R     = 0.22
MAX_REACH  = 0.62
MOVE_SECS  = 3.0
HOLD_SECS  = 1.5
APPROACH_Z = TABLE_Z + 0.40
LIFT_Z     = TABLE_Z + 0.40
DROP_XY    = np.array([0.65, -0.3])
SNAP_GAP   = 0.055
R_DOWN = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])

_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}
_ACT_NAMES   = [f"act_a{i}" for i in range(1, 7)]
_JOINT_NAMES = [f"joint_a{i}" for i in range(1, 7)]

_SAFE_RANGES = [
    (-2.97,  2.97),
    (-1.40,  0.50),
    (-0.50,  2.00),
    (-3.14,  3.14),
    (-1.50,  1.50),
    (-3.14,  3.14),
]
READY_Q = np.array([0.0, -1.05, 1.65, 0.0, 0.96, 0.0])


def _random_box(rng):
    for _ in range(2000):
        bx = rng.uniform(-0.60, 0.60)
        by = rng.uniform(-0.25, 0.28)
        r  = float(np.linalg.norm([bx - ARM_BASE[0], by - ARM_BASE[1]]))
        if EXCL_R < r <= MAX_REACH:
            return np.array([bx, by, TABLE_Z + BOX_HALF])
    raise RuntimeError("Cannot sample a valid box position.")


def _solve_ik(model, qpas, dofas, ranges, site_id, seed_q, tgt_pos):
    d  = mujoco.MjData(model)
    nv = model.nv
    for qi, q in zip(qpas, seed_q):
        d.qpos[qi] = q
    mujoco.mj_forward(model, d)
    for _ in range(1200):
        jp = np.zeros((3, nv)); jr = np.zeros((3, nv))
        mujoco.mj_forward(model, d)
        mujoco.mj_jacSite(model, d, jp, jr, site_id)
        ep = tgt_pos - d.site_xpos[site_id]
        R  = R_DOWN @ d.site_xmat[site_id].reshape(3, 3).T
        er = 0.5 * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
        J  = np.vstack([jp, jr])
        e  = np.concatenate([ep, er * 0.5])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), e)
        for i, (qi, di) in enumerate(zip(qpas, dofas)):
            d.qpos[qi] = np.clip(d.qpos[qi] + dq[di] * 0.05, *ranges[i])
        if float(np.linalg.norm(ep)) < 3e-3:
            break
    return np.array([d.qpos[qi] for qi in qpas])


def main():
    pathlib.Path(OUTPUT_MP4).parent.mkdir(parents=True, exist_ok=True)

    print(f"Rendering backend: MUJOCO_GL={os.environ.get('MUJOCO_GL', 'not set')}")

    model    = mujoco.MjModel.from_xml_path(WORLD_XML)
    data     = mujoco.MjData(model)
    apply_gains(model, _GAINS)
    DT = model.opt.timestep

    # offscreen renderer — one shared renderer for all 3 cameras
    renderer = mujoco.Renderer(model, height=CAM_H, width=CAM_W)

    jids    = [model.joint(n).id         for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j]      for j in jids]
    dofas   = [model.jnt_dofadr[j]       for j in jids]
    ranges  = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id      for n in _ACT_NAMES]
    site_id     = model.site("em_contact_site").id
    box_site_id = model.site("box_top_site").id
    box_jid = model.joint("box_freejoint").id
    bqpa    = model.jnt_qposadr[box_jid]

    # How often to capture a frame: 1 frame per VIDEO_STEP physics steps
    VIDEO_STEP = max(1, int(1.0 / (VIDEO_FPS * DT)))  # ≈33 for 30fps at 1ms

    move_steps = max(1, int(MOVE_SECS / DT))
    hold_steps = max(1, int(HOLD_SECS / DT))

    def _capture():
        """Render all 3 cameras and return a single wide RGB frame."""
        mujoco.mj_forward(model, data)   # ensure scene is up-to-date before render
        strips = []
        for cam in CAMS:
            renderer.update_scene(data, camera=cam)
            strips.append(renderer.render().copy())
        return np.concatenate(strips, axis=1)  # shape: (CAM_H, CAM_W*3, 3)

    # ── phase helpers ─────────────────────────────────────────────────────────
    frames = []
    step_n = [0]

    def _step_and_capture():
        mujoco.mj_step(model, data)
        step_n[0] += 1
        if step_n[0] % VIDEO_STEP == 0:
            frames.append(_capture())

    def _lerp_phase(q_from, q_to):
        for i in range(move_steps):
            frac = smooth(i / move_steps)
            for ai, v0, v1 in zip(act_ids, q_from, q_to):
                data.ctrl[ai] = v0 + frac * (v1 - v0)
            _step_and_capture()

    def _hold_phase(q):
        for _ in range(hold_steps):
            for ai, v in zip(act_ids, q):
                data.ctrl[ai] = v
            _step_and_capture()

    def smooth(t):
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    # ── single cycle ─────────────────────────────────────────────────────────
    rng = np.random.default_rng(42)  # fixed seed for reproducibility

    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)

    arm_body_ids = [model.body(n).id for n in
                    ["link_1","link_2","link_3","link_4","link_5","link_6","em_pad"]]

    def arm_above_table(q):
        d_tmp = mujoco.MjData(model)
        for qi, v in zip(qpas, q):
            d_tmp.qpos[qi] = v
        mujoco.mj_forward(model, d_tmp)
        return all(d_tmp.xpos[bid, 2] >= TABLE_Z for bid in arm_body_ids)

    # random arm pose
    for _ in range(500):
        arm_q = np.array([rng.uniform(lo, hi) for lo, hi in _SAFE_RANGES])
        if arm_above_table(arm_q):
            break
    else:
        arm_q = np.zeros(6)

    for qi, q in zip(qpas, arm_q):
        data.qpos[qi] = q
    for ai, q in zip(act_ids, arm_q):
        data.ctrl[ai] = q

    box_pos = _random_box(rng)
    data.qpos[bqpa:bqpa + 7] = [*box_pos, 1., 0., 0., 0.]
    mujoco.mj_forward(model, data)

    dx = box_pos[0] - ARM_BASE[0]
    dy = box_pos[1] - ARM_BASE[1]
    a1_lo, a1_hi = model.jnt_range[jids[0]]
    a1_target    = float(np.clip(math.atan2(-dy, dx), a1_lo, a1_hi))

    if abs(a1_target) <= math.pi / 2:
        hub_a1 = 0.0
    else:
        hub_a1 = float(np.clip(math.copysign(math.pi, a1_target), a1_lo, a1_hi))

    hub_q     = READY_Q.copy(); hub_q[0] = hub_a1
    aligned_q = READY_Q.copy(); aligned_q[0] = a1_target

    print("Pre-solving IK...", end=" ", flush=True)
    above_tgt = np.array([box_pos[0], box_pos[1], APPROACH_Z])
    above_q   = _solve_ik(model, qpas, dofas, ranges, site_id, aligned_q, above_tgt)
    snap_z    = box_pos[2] + BOX_HALF + SNAP_GAP
    snap_tgt  = np.array([box_pos[0], box_pos[1], snap_z])
    print("done")

    t_start = time.time()
    print("Recording... (this will take a minute)")

    # P0→hub
    _lerp_phase(arm_q, hub_q);   _hold_phase(hub_q)
    # hub→P3
    _lerp_phase(hub_q, aligned_q); _hold_phase(aligned_q)
    # P4 ABOVE_BOX
    _lerp_phase(aligned_q, above_q); _hold_phase(above_q)
    # P5 SNAP
    actual_q = np.array([data.qpos[qi] for qi in qpas])
    snap_q   = _solve_ik(model, qpas, dofas, ranges, site_id, actual_q, snap_tgt)
    _lerp_phase(above_q, snap_q); _hold_phase(snap_q)
    # P6 GRAB
    grabbed = False
    for _ in range(hold_steps):
        for ai, v in zip(act_ids, snap_q):
            data.ctrl[ai] = v
        _step_and_capture()
        if not grabbed and em_activate(model, data, threshold_m=0.06):
            grabbed = True
    print(f"  GRAB: {'OK' if grabbed else 'FAILED'}")
    # P7 LIFT
    actual_q = np.array([data.qpos[qi] for qi in qpas])
    lift_q   = _solve_ik(model, qpas, dofas, ranges, site_id, actual_q,
                         np.array([box_pos[0], box_pos[1], LIFT_Z]))
    _lerp_phase(snap_q, lift_q); _hold_phase(lift_q)
    # P8 CARRY
    actual_q = np.array([data.qpos[qi] for qi in qpas])
    carry_q  = _solve_ik(model, qpas, dofas, ranges, site_id, actual_q,
                         np.array([DROP_XY[0], DROP_XY[1], LIFT_Z]))
    _lerp_phase(lift_q, carry_q); _hold_phase(carry_q)
    # P9 DROP
    em_deactivate(model, data)
    _hold_phase(carry_q)

    elapsed = time.time() - t_start
    print(f"Simulation done in {elapsed:.1f}s  ({len(frames)} frames captured)")

    # ── write MP4 ─────────────────────────────────────────────────────────────
    print(f"Writing {OUTPUT_MP4} ...", end=" ", flush=True)

    try:
        import cv2
        def add_labels(frame):
            out = frame.copy()
            label_h = 24
            for i, name in enumerate(CAMS):
                x0 = i * CAM_W
                cv2.rectangle(out, (x0, 0), (x0 + CAM_W, label_h), (20, 20, 20), -1)
                cv2.putText(out, name, (x0 + 6, label_h - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
            return out
        labeled_frames = [add_labels(f) for f in frames]
    except ImportError:
        labeled_frames = frames  # no cv2 — skip labels

    with imageio.get_writer(OUTPUT_MP4, fps=VIDEO_FPS, quality=8,
                            codec="libx264", pixelformat="yuv420p") as writer:
        for f in labeled_frames:
            writer.append_data(f)

    print(f"done  ({len(labeled_frames)} frames, {len(labeled_frames)/VIDEO_FPS:.1f}s)")
    print(f"Saved → {OUTPUT_MP4}")


if __name__ == "__main__":
    main()
