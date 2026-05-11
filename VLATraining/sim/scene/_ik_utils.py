"""
scene/_ik_utils.py  —  Phase 10 shared IK utilities
═══════════════════════════════════════════════════════════════════════════════
Damped-least-squares (DLS) Jacobian IK shared by all Phase 10 demo scripts.
Import pattern:

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
    from scene._ik_utils import R_DOWN, _ik_step, _run_ik, _sync_ctrl, _a1_toward

No side-effects on import.  All functions are pure or operate on mujoco data.
"""

import math

import mujoco
import numpy as np


# ── Target orientation: EM face (site local +X) pointing straight down ────────
#
#   site_xmat is column-major [col0 | col1 | col2].
#   We want site local-X axis to point in world −Z (down):
#       R[:,0] = [ 0,  0, −1]   ← site X in world = world −Z
#       R[:,1] = [ 0,  1,  0]   ← site Y in world = world +Y (unchanged)
#       R[:,2] = [+1,  0,  0]   ← site Z in world = world +X
#   det(R) = +1 ✓
#
R_DOWN = np.array([
    [ 0.,  0.,  1.],
    [ 0.,  1.,  0.],
    [-1.,  0.,  0.],
])


def _ik_step(model, data, dofas, qpas, ranges,
             site_id, target_pos, target_mat=None,
             step=0.05, lam=1e-3):
    """
    One DLS IK step toward target_pos (and optional target_mat orientation).

    Parameters
    ----------
    model, data   : MuJoCo model and data objects.
    dofas, qpas   : DOF-address and qpos-address lists for the arm joints.
    ranges        : List of (lo, hi) joint limit tuples.
    site_id       : Integer ID of the control site (em_contact_site).
    target_pos    : (3,) world-frame position target.
    target_mat    : (3,3) desired rotation matrix for the site.
                    None → position-only IK.
    step          : Damping step size applied to Δq.
    lam           : DLS regularisation weight (λ²I).

    Returns
    -------
    float : Position error magnitude (m) after this step.
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
        # Rotation-error vector (world frame, small-angle):
        #   er = 0.5 * vee(R_err − R_err^T)
        er = 0.5 * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])
        J = np.vstack([jacp, jacr])
        e = np.concatenate([ep, er * 0.5])   # orientation at 0.5× weight
    else:
        J = jacp
        e = ep

    JJT = J @ J.T + lam * np.eye(J.shape[0])
    dq  = J.T @ np.linalg.solve(JJT, e)

    for i, (qi, di) in enumerate(zip(qpas, dofas)):
        lo, hi = ranges[i]
        data.qpos[qi] = np.clip(data.qpos[qi] + dq[di] * step, lo, hi)

    return float(np.linalg.norm(ep))


def _run_ik(model, data, dofas, qpas, ranges, site_id,
            target_pos, target_mat=None,
            max_iter=600, pos_tol=5e-3, step=0.05, lam=1e-3):
    """
    Run IK until position error < pos_tol or max_iter reached.

    Returns
    -------
    (converged : bool, final_pos_error : float)
    """
    err = float("inf")
    for _ in range(max_iter):
        err = _ik_step(model, data, dofas, qpas, ranges,
                       site_id, target_pos, target_mat, step, lam)
        if err < pos_tol:
            return True, err
    return False, err


def _sync_ctrl(data, qpas, act_ids):
    """Set PD controller targets to current IK-solved joint positions."""
    for qi, ai in zip(qpas, act_ids):
        data.ctrl[ai] = data.qpos[qi]


def _a1_toward(box_pos, arm_base, ranges):
    """
    Analytic A1 angle pointing the arm base toward box_pos XY.

    At A1=θ the arm extends in direction [cos θ, −sin θ, 0] (world XY).
    Solve: θ = atan2(−dy, dx)  where (dx, dy) = box_pos[:2] − arm_base.

    Parameters
    ----------
    box_pos  : (3,) or (2,) target XY (only first 2 elements used).
    arm_base : (2,) arm base XY in world.
    ranges   : List of (lo, hi) for all joints; ranges[0] is A1's limits.

    Returns
    -------
    float : Clamped A1 target angle (rad).
    """
    dx = float(box_pos[0]) - float(arm_base[0])
    dy = float(box_pos[1]) - float(arm_base[1])
    angle = math.atan2(-dy, dx)
    lo, hi = ranges[0]
    return float(np.clip(angle, lo, hi))
