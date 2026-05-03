"""
scene/demo_pickup_throw.py  —  Phase 6 Bonus Demo
══════════════════════════════════════════════════════════════════════════════
Full pick-and-throw sequence with randomised initial conditions.

Sequence
─────────
 1.  Random box position on table — clear of arm base (r > 0.20 m from
     arm XY base; r < 0.70 m for reliable IK reachability).
 2.  Random valid arm start pose — sampled in joint space, rejected if
     EE ends up below table + 0.15 m.
 3.  IK + physics: raise EE to clearance height (≥ table + 3 × box_h),
     EM face pointing straight down (−Z world).
 4.  IK + physics: translate EE horizontally directly over the box,
     EM face still pointing down.
 5.  IK + physics: descend to snap height (box_top + 1.5 cm gap),
     EM face pointing down; box pinned so it does not drift.
 6.  em_activate() — weld box to arm.
 7.  IK + physics: lift box to throw height.
 8.  Joint-space swing — rotate A1 outward ~90°; release EM once
     the arm is moving fast enough (|ω_A1| > 0.4 rad/s).
 9.  Continue sim — box flies and lands.
10.  Save video → outputs/demo_pickup_throw.mp4

IK method: damped least-squares (DLS) Jacobian pseudoinverse.
  Position + orientation targets use the 6×nv stacked Jacobian.
  Orientation error weight is 0.5× position to keep position primary.

Run:
  source phase4_env/bin/activate
  PYTHONPATH=VLATraining/sim python VLATraining/sim/scene/demo_pickup_throw.py
"""

import math
import pathlib
import sys

import imageio
import mujoco
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate

# ── File paths ────────────────────────────────────────────────────────────────
_SIM_DIR  = pathlib.Path(__file__).parent.parent          # VLATraining/sim/
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
OUT_VIDEO = str(_SIM_DIR / "outputs" / "demo_pickup_throw.mp4")

# ── Scene geometry ────────────────────────────────────────────────────────────
TABLE_Z   = 0.85    # tabletop surface (world Z, m)
BOX_HALF  = 0.05    # box half-size (m);  full height  h = 0.10 m
BOX_H     = 2 * BOX_HALF
SNAP_GAP  = 0.015   # gap between box top and EM face at grab

ARM_BASE  = np.array([0.0, -0.3])  # arm base XY in world (from arm.xml)

# ── Motion parameters ─────────────────────────────────────────────────────────
EXCL_R      = 0.20   # min XY distance from arm base (arm body exclusion zone)
MAX_REACH   = 0.70   # max XY distance from arm base for reliable IK (m)
APPROACH_Z  = TABLE_Z + 3.0 * BOX_H + 0.05   # clearance height = 1.20 m
THROW_Z     = TABLE_Z + 0.55                  # lift height before throw = 1.40 m
THROW_SWING = 1.57                            # A1 swing angle for throw (~90°)

# ── Render / video ────────────────────────────────────────────────────────────
RENDER_SKIP = 16    # render every N physics steps (1000 Hz / 16 = 62.5 fps)
VID_W, VID_H = 1280, 720

# ── PD gains (same as all other Phase 6 scripts) ─────────────────────────────
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

# ── Target orientation: EM face (site local +X) pointing straight down ────────
#
#   site_xmat is a 3×3 rotation matrix R where R[:,k] = site's local k-axis
#   in world frame.  We want:
#       R[:,0] = [0, 0, −1]   (local X → world −Z = straight down)
#       R[:,1] = [0, 1,  0]   (local Y unchanged)
#       R[:,2] = [1, 0,  0]   (local Z = cross product)
#   det = +1  ✓  (proper rotation)
#
R_DOWN = np.array([
    [ 0.,  0.,  1.],
    [ 0.,  1.,  0.],
    [-1.,  0.,  0.],
])


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Model loading                                                               ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def _load_model():
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids    = [model.joint(n).id  for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j] for j in jids]   # qpos addresses
    dofas   = [model.jnt_dofadr[j]  for j in jids]   # DOF  addresses
    ranges  = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    site_id = model.site("em_contact_site").id

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]
    box_doa = model.jnt_dofadr[box_jid]

    return model, data, qpas, dofas, ranges, act_ids, site_id, box_qpa, box_doa


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Numerical IK  —  damped least-squares                                       ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def _ik_step(model, data, dofas, qpas, ranges,
             site_id, target_pos, target_mat=None,
             step=0.05, lam=1e-3):
    """
    One DLS IK step.

    target_mat: desired 3×3 rotation matrix for the site (R_DOWN for
                "pointing down").  Pass None for position-only IK.

    Returns the position error magnitude (m).
    """
    nv   = model.nv
    jacp = np.zeros((3, nv))
    jacr = np.zeros((3, nv))

    mujoco.mj_forward(model, data)
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)

    ep = target_pos - data.site_xpos[site_id]

    if target_mat is not None:
        R_cur = data.site_xmat[site_id].reshape(3, 3)
        R_err = target_mat @ R_cur.T
        # Rotation-error vector (world frame, small-angle approximation):
        #   er = vee(skew(R_err)) = 0.5 * (R_err - R_err^T)^∨
        er = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])
        J = np.vstack([jacp, jacr])
        # Weight orientation at 0.5× position so position is primary target
        e = np.concatenate([ep, er * 0.5])
    else:
        J = jacp
        e = ep

    JJT = J @ J.T + lam * np.eye(J.shape[0])
    dq  = J.T @ np.linalg.solve(JJT, e)

    # Update only arm joints; clamp to joint limits
    for i, (qi, di) in enumerate(zip(qpas, dofas)):
        lo, hi = ranges[i]
        data.qpos[qi] = np.clip(data.qpos[qi] + dq[di] * step, lo, hi)

    return float(np.linalg.norm(ep))


def _run_ik(model, data, dofas, qpas, ranges, site_id,
            target_pos, target_mat=None,
            max_iter=600, pos_tol=5e-3, step=0.05, lam=1e-3):
    """
    Run IK until position error < pos_tol or max_iter reached.
    Returns (converged: bool, final_pos_error: float).
    """
    err = float("inf")
    for _ in range(max_iter):
        err = _ik_step(model, data, dofas, qpas, ranges,
                       site_id, target_pos, target_mat, step, lam)
        if err < pos_tol:
            return True, err
    return False, err


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Utilities                                                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def _sync_ctrl(data, qpas, act_ids):
    """Set PD controller targets to current IK-solved qpos (hold in place)."""
    for qi, ai in zip(qpas, act_ids):
        data.ctrl[ai] = data.qpos[qi]


def _random_box_pos(rng):
    """
    Sample a random box centre on the table surface.
    Constraints:
      • Inside table area (X ∈ [−0.65, 0.65], Y ∈ [−0.28, 0.30])
      • Outside arm-base exclusion circle  (r > EXCL_R = 0.20 m)
      • Within reliable IK reach           (r ≤ MAX_REACH = 0.70 m)
    """
    for _ in range(2000):
        bx = rng.uniform(-0.65, 0.65)
        by = rng.uniform(-0.28, 0.30)
        r  = np.linalg.norm([bx - ARM_BASE[0], by - ARM_BASE[1]])
        if EXCL_R < r <= MAX_REACH:
            return np.array([bx, by, TABLE_Z + BOX_HALF])
    raise RuntimeError("Could not find valid box position after 2000 tries.")


def _random_arm_start(model, data, rng, qpas, dofas, ranges,
                      act_ids, site_id):
    """
    Sample a random joint-space arm pose.
    Acceptance criterion: EE Z > TABLE_Z + 0.15 m  (arm clearly above table).
    Limits are biased to 70% to avoid extreme / degenerate configurations.
    Falls back to a known safe overhead pose if sampling fails.
    Returns True if a random pose was found, False if fallback was used.
    """
    for _ in range(300):
        for i, (qi, (lo, hi)) in enumerate(zip(qpas, ranges)):
            lo_b = lo * 0.7
            hi_b = hi * 0.7
            data.qpos[qi] = rng.uniform(lo_b, hi_b)
        mujoco.mj_forward(model, data)
        if data.site_xpos[site_id][2] > TABLE_Z + 0.15:
            _sync_ctrl(data, qpas, act_ids)
            return True

    # Fallback: overhead-reach pose — arm pointing forward and slightly up
    _SAFE_DEG = [0, -45, 60, 0, 30, 0]
    for qi, deg in zip(qpas, _SAFE_DEG):
        data.qpos[qi] = math.radians(deg)
    _sync_ctrl(data, qpas, act_ids)
    mujoco.mj_forward(model, data)
    return False


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  Main demo                                                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

def main():
    rng = np.random.default_rng()

    # ── Load model ─────────────────────────────────────────────────────────────
    (model, data, qpas, dofas, ranges,
     act_ids, site_id, box_qpa, box_doa) = _load_model()

    # ── Open streaming video writer (no frame list → O(1) memory) ────────────
    pathlib.Path(OUT_VIDEO).parent.mkdir(parents=True, exist_ok=True)
    fps = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    _writer = imageio.get_writer(OUT_VIDEO, fps=fps)

    # ─────────────────────────────────────────────────────────────────────────
    # 1.  Random box position
    # ─────────────────────────────────────────────────────────────────────────
    box_pos  = _random_box_pos(rng)
    box_init = np.array([box_pos[0], box_pos[1], box_pos[2],
                         1., 0., 0., 0.])   # [x, y, z, qw, qx, qy, qz]
    print(f"[1] Box centre : ({box_pos[0]:.3f}, {box_pos[1]:.3f}, {box_pos[2]:.3f})")

    # ─────────────────────────────────────────────────────────────────────────
    # 2.  Random arm start
    # ─────────────────────────────────────────────────────────────────────────
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)
    data.qpos[box_qpa:box_qpa + 7] = box_init
    mujoco.mj_forward(model, data)

    fallback = not _random_arm_start(model, data, rng, qpas, dofas, ranges,
                                     act_ids, site_id)
    angles = [round(math.degrees(data.qpos[qi]), 1) for qi in qpas]
    ee0    = data.site_xpos[site_id].copy()
    print(f"[2] Arm start  : {angles}°  (fallback={fallback})")
    print(f"    EE start   : ({ee0[0]:.3f}, {ee0[1]:.3f}, {ee0[2]:.3f})")

    # Settle at start pose (1 s, box free on table)
    for _ in range(1000):
        mujoco.mj_step(model, data)

    # ─────────────────────────────────────────────────────────────────────────
    # Renderer setup  — frames are written to disk immediately (streaming)
    # ─────────────────────────────────────────────────────────────────────────
    renderer = mujoco.Renderer(model, height=VID_H, width=VID_W)
    _step_n  = [0]

    def _physics_step():
        mujoco.mj_step(model, data)
        if _step_n[0] % RENDER_SKIP == 0:
            renderer.update_scene(data)
            _writer.append_data(renderer.render())   # write, then discard
        _step_n[0] += 1

    def _run_n(n):
        for _ in range(n):
            _physics_step()

    # ─────────────────────────────────────────────────────────────────────────
    # 3.  Rise to clearance height, EM pointing down
    # ─────────────────────────────────────────────────────────────────────────
    print(f"\n[3] IK: rise to clearance Z = {APPROACH_Z:.2f} m  (EM pointing down) …")
    ee_xy = data.site_xpos[site_id][:2].copy()
    ok, err = _run_ik(model, data, dofas, qpas, ranges,
                      site_id, np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1000)   # 1.0 s settle

    # ─────────────────────────────────────────────────────────────────────────
    # 3b. Pre-align A1 toward box  (analytic — avoids IK getting stuck when
    #     the arm needs to swing A1 > 90° to face the box)
    #
    #     At A1=θ the arm extends in direction [cos θ, −sin θ, 0] (world XY).
    #     Solve for θ given the box XY relative to arm base:
    #         cos θ = dx / r,  −sin θ = dy / r
    #         →  θ = atan2(−dy, dx)   where (dx, dy) = box_pos[:2] − ARM_BASE
    # ─────────────────────────────────────────────────────────────────────────
    dx, dy  = box_pos[0] - ARM_BASE[0], box_pos[1] - ARM_BASE[1]
    a1_tgt  = math.atan2(-dy, dx)
    a1_tgt  = float(np.clip(a1_tgt, *ranges[0]))
    print(f"[3b] Pre-align A1 → {math.degrees(a1_tgt):.1f}° to face box …")
    data.qpos[qpas[0]]  = a1_tgt
    data.ctrl[act_ids[0]] = a1_tgt
    _run_n(2000)   # 2.0 s to physically swing A1

    # ─────────────────────────────────────────────────────────────────────────
    # 4.  Translate over box, EM pointing down
    # ─────────────────────────────────────────────────────────────────────────
    print(f"[4] IK: translate over box  ({box_pos[0]:.3f}, {box_pos[1]:.3f})  …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges,
                      site_id, np.array([box_pos[0], box_pos[1], APPROACH_Z]),
                      R_DOWN, max_iter=800)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)   # 1.5 s settle

    # ─────────────────────────────────────────────────────────────────────────
    # 5.  Descend to snap height, EM pointing down
    #     Box is pinned each step so it does not drift before grab.
    # ─────────────────────────────────────────────────────────────────────────
    snap_z   = box_pos[2] + BOX_HALF + SNAP_GAP   # box_top + 1.5 cm gap
    print(f"[5] IK: descend to snap Z = {snap_z:.3f} m  (EM pointing down) …")
    ok, err = _run_ik(model, data, dofas, qpas, ranges,
                      site_id, np.array([box_pos[0], box_pos[1], snap_z]),
                      R_DOWN, pos_tol=3e-3)
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)

    # Settle with box pinned (established Phase-6 pattern)
    for _ in range(1500):
        data.qpos[box_qpa:box_qpa + 7] = box_init
        data.qvel[box_doa:box_doa + 6] = 0.
        mujoco.mj_step(model, data)
        if _step_n[0] % RENDER_SKIP == 0:
            renderer.update_scene(data)
            _writer.append_data(renderer.render())
        _step_n[0] += 1

    # ─────────────────────────────────────────────────────────────────────────
    # 6.  Activate EM
    # ─────────────────────────────────────────────────────────────────────────
    data.qvel[:] = 0.
    data.qpos[box_qpa:box_qpa + 7] = box_init
    mujoco.mj_forward(model, data)
    activated = em_activate(model, data, threshold_m=0.05)
    print(f"[6] EM activate: {activated}")
    if not activated:
        print("    WARNING: EM did not activate — EE may be too far from box.")
    _run_n(300)    # 0.3 s settle with box attached

    # ─────────────────────────────────────────────────────────────────────────
    # 7.  Lift to throw height  (position-only IK — no orientation constraint)
    # ─────────────────────────────────────────────────────────────────────────
    print(f"[7] IK: lift to throw height Z = {THROW_Z:.2f} m …")
    ee_xy_now = data.site_xpos[site_id][:2].copy()
    ok, err   = _run_ik(model, data, dofas, qpas, ranges,
                        site_id, np.array([ee_xy_now[0], ee_xy_now[1], THROW_Z]))
    print(f"    IK {'converged' if ok else 'partial'}, pos_err = {err:.4f} m")
    _sync_ctrl(data, qpas, act_ids)
    _run_n(1500)   # 1.5 s lift

    # ─────────────────────────────────────────────────────────────────────────
    # 8.  Swing A1 outward and release
    # ─────────────────────────────────────────────────────────────────────────
    print("[8] Throwing …")
    a1_qi       = qpas[0]
    a1_di       = dofas[0]
    a1_lo, a1_hi = ranges[0]
    cur_a1      = data.qpos[a1_qi]

    # Swing in the direction that has more room within joint limits
    room_pos = a1_hi - cur_a1
    room_neg = cur_a1 - a1_lo
    swing    = +THROW_SWING if room_pos >= room_neg else -THROW_SWING
    throw_target = np.clip(cur_a1 + swing, a1_lo, a1_hi)
    data.ctrl[act_ids[0]] = throw_target
    print(f"    A1: {math.degrees(cur_a1):.1f}° → {math.degrees(throw_target):.1f}°")

    released = False
    for _ in range(2500):
        _physics_step()
        if not released and abs(data.qvel[a1_di]) > 0.4:
            em_deactivate(model, data)
            released = True
            print(f"    Released  A1 = {math.degrees(data.qpos[a1_qi]):.1f}°, "
                  f"ω_A1 = {data.qvel[a1_di]:.2f} rad/s")

    if not released:
        em_deactivate(model, data)
        print("    Released (timeout — velocity threshold not reached)")

    # ─────────────────────────────────────────────────────────────────────────
    # 9.  Box lands
    # ─────────────────────────────────────────────────────────────────────────
    print("[9] Box landing …")
    _run_n(2500)

    # ─────────────────────────────────────────────────────────────────────────
    # 10. Finalise video (already written — just close the writer)
    # ─────────────────────────────────────────────────────────────────────────
    _writer.close()
    total_frames = _step_n[0] // RENDER_SKIP
    print(f"\nVideo saved  ({total_frames} frames @ {fps} fps) → {OUT_VIDEO}")
    print("Done ✓")


if __name__ == "__main__":
    main()
