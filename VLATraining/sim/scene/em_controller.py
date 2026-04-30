"""
scene/em_controller.py
──────────────────────────────────────────────────────────────────────────
Phase 6 — Electromagnetic attachment controller (Tasks 6.5 and 6.6).

API
───
  em_can_activate(data, model, threshold_m=0.05) → bool
      True when em_contact_site is within threshold_m of box_top_site.

  em_activate(model, data) → bool
      Checks proximity; if OK computes the CURRENT relative pose of
      metal_box w.r.t. link_6 and writes it into the weld eq_data so
      the box is locked exactly where it is — no snap, no rotation.
      Returns True if the weld was activated, False if out of range.

  em_deactivate(model, data) → True
      Unconditionally disables the weld.
"""

import numpy as np
import mujoco


def em_can_activate(data: mujoco.MjData,
                    model: mujoco.MjModel,
                    threshold_m: float = 0.05) -> bool:
    """
    Returns True when em_contact_site is within threshold_m of box_top_site.
    No orientation check — any approach angle is accepted.
    """
    em_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "em_contact_site")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "box_top_site")
    if em_id < 0:
        raise RuntimeError("em_contact_site not found in model.")
    if box_id < 0:
        raise RuntimeError("box_top_site not found in model.")

    dist = float(np.linalg.norm(data.site_xpos[em_id] - data.site_xpos[box_id]))
    return dist < threshold_m


def em_activate(model: mujoco.MjModel,
                data: mujoco.MjData,
                threshold_m: float = 0.05) -> bool:
    """
    Activate the em_weld constraint if in range.

    Computes the current relative pose of metal_box w.r.t. link_6
    and stores it in model.eq_data — so the constraint is satisfied
    immediately with zero residual (box stays flat, no rotation snap).

    Returns True if activated, False if out of range.
    """
    if not em_can_activate(data, model, threshold_m=threshold_m):
        return False

    weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
    if weld_id < 0:
        raise RuntimeError("em_weld equality constraint not found in model.")

    em_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "em_pad")
    box_id    = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")

    # Current world poses
    mat_em   = data.xmat[em_pad_id].reshape(3, 3)
    mat_box  = data.xmat[box_id].reshape(3, 3)
    quat_l6  = data.xquat[em_pad_id].copy()   # [w, x, y, z]
    quat_box = data.xquat[box_id].copy()

    # Relative position: box origin in em_pad frame, such that
    # box_top_site ([0,0,0.05] in box frame) aligns with
    # em_contact_site ([0.01,0,0] in em_pad frame).
    # Derivation:  em_pad_origin + R_em @ rel_pos = box_center
    #              box_center = em_site_world - R_box @ [0,0,0.05]
    #              em_site_world = em_pad_origin + R_em @ [0.01,0,0]
    # => rel_pos = [0.01,0,0] - R_em.T @ R_box @ [0,0,0.05]
    EM_SITE_IN_EM_FRAME  = np.array([0.01, 0.0, 0.0])   # from arm.xml
    BOX_TOP_IN_BOX_FRAME = np.array([0.0,  0.0, 0.05])  # from objects.xml
    rel_pos = EM_SITE_IN_EM_FRAME - mat_em.T @ (mat_box @ BOX_TOP_IN_BOX_FRAME)

    # Relative quaternion: q_rel = inv(q_l6) * q_box
    def quat_inv(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def quat_mul(a, b):
        w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
        x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
        y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
        z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
        return np.array([w, x, y, z])

    rel_quat = quat_mul(quat_inv(quat_l6), quat_box)

    # Write relpose into eq_data layout (11 elements per weld):
    #   [0:3]  = relpos   — body2 origin position in body1 frame
    #   [3:6]  = anchor2  — attachment point in body2 frame (box origin = [0,0,0])
    #   [6:10] = relquat  — relative orientation (body2 relative to body1)
    #   [10]   = torquescale (leave unchanged)
    model.eq_data[weld_id, 0:3] = rel_pos
    model.eq_data[weld_id, 3:6] = [0.0, 0.0, 0.0]   # anchor2 = box origin
    model.eq_data[weld_id, 6:10] = rel_quat

    # Activate
    model.eq_active0[weld_id] = 1
    data.eq_active[weld_id]   = 1
    return True


def em_deactivate(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    """Unconditionally disable the em_weld. Always returns True."""
    weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
    if weld_id < 0:
        raise RuntimeError("em_weld equality constraint not found in model.")
    model.eq_active0[weld_id] = 0
    data.eq_active[weld_id]   = 0
    return True
