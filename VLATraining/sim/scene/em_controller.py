"""
scene/em_controller.py
──────────────────────────────────────────────────────────────────────────
Phase 6 — Electromagnetic attachment controller (Tasks 6.5 and 6.6).

Design: "magnetic connection already solved."
  em_activate() does not simulate magnetic attraction physics.
  When preconditions are met it snaps the box to the ideal attached
  geometry — top face flush with EM face, box world-upright — and
  locks the weld at that exact pose. No tilt inherited from imprecise
  approach angle, no gap preserved from the threshold distance.

API
───
  em_can_activate(data, model, threshold_m=0.02) → bool
      True when em_contact_site is within threshold_m of box_top_site
      AND the box is approximately upright (top face within 20° of world Z).

  em_activate(model, data) → bool
      Checks proximity + orientation; if OK, snaps the box to the ideal
      attached pose, writes that into eq_data, and activates the weld.
      Returns True if activated, False if preconditions failed.

  em_deactivate(model, data) → True
      Unconditionally disables the weld.
"""

import numpy as np
import mujoco

# Site offsets in their respective local body frames (from arm.xml / objects.xml)
_EM_SITE_LOCAL = np.array([0.01, 0.0, 0.0])   # em_contact_site in em_pad frame
_BOX_TOP_LOCAL = np.array([0.0,  0.0, 0.05])   # box_top_site in metal_box frame
_UPRIGHT_QUAT  = np.array([1.0,  0.0, 0.0, 0.0])  # identity [w, x, y, z]


def em_can_activate(data: mujoco.MjData,
                    model: mujoco.MjModel,
                    threshold_m: float = 0.02) -> bool:
    """
    Returns True when:
      1. em_contact_site is within threshold_m of box_top_site.
      2. The box is approximately upright (local Z within 20° of world Z).
    """
    em_id      = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "em_contact_site")
    box_sit_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "box_top_site")
    if em_id < 0:
        raise RuntimeError("em_contact_site not found in model.")
    if box_sit_id < 0:
        raise RuntimeError("box_top_site not found in model.")

    # Proximity check
    dist = float(np.linalg.norm(data.site_xpos[em_id] - data.site_xpos[box_sit_id]))
    if dist >= threshold_m:
        return False

    # Orientation check: box local Z must be within 20° of world Z
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    box_z_world = data.xmat[box_bid].reshape(3, 3)[:, 2]  # third column = local Z in world
    if box_z_world[2] < np.cos(np.radians(20)):
        return False

    return True


def em_activate(model: mujoco.MjModel,
                data: mujoco.MjData,
                threshold_m: float = 0.02) -> bool:
    """
    Activate the em_weld constraint if preconditions are met.

    Snaps the box to the ideal attached pose (top site flush with EM contact
    site, box world-upright), writes that pose into eq_data, then activates.

    Returns True if activated, False if preconditions failed.
    """
    if not em_can_activate(data, model, threshold_m=threshold_m):
        return False

    weld_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
    em_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "em_pad")
    box_jnt   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    "box_freejoint")
    if weld_id < 0:
        raise RuntimeError("em_weld equality constraint not found in model.")
    if box_jnt < 0:
        raise RuntimeError("box_freejoint not found in model.")

    mat_em = data.xmat[em_pad_id].reshape(3, 3)
    p_em   = data.xpos[em_pad_id].copy()

    # World position of em_contact_site
    p_site = p_em + mat_em @ _EM_SITE_LOCAL

    # Ideal box world position: top site coincides with EM contact site,
    # box is world-upright so _BOX_TOP_LOCAL is just [0, 0, 0.05] in world frame
    p_box_ideal = p_site - _BOX_TOP_LOCAL

    # Snap box to ideal pose, zero velocity
    qpa = model.jnt_qposadr[box_jnt]
    doa = model.jnt_dofadr[box_jnt]
    data.qpos[qpa:qpa+3]   = p_box_ideal
    data.qpos[qpa+3:qpa+7] = _UPRIGHT_QUAT
    data.qvel[doa:doa+6]   = 0.0
    mujoco.mj_forward(model, data)

    # Ideal relpose in em_pad frame:
    #   rel_pos  = R_em.T @ (p_box_ideal - p_em)
    #            = _EM_SITE_LOCAL - R_em.T @ _BOX_TOP_LOCAL
    #   rel_quat = inv(q_em)  →  box frame is world-upright expressed in em_pad frame
    rel_pos  = _EM_SITE_LOCAL - mat_em.T @ _BOX_TOP_LOCAL
    rel_quat = np.zeros(4)
    mujoco.mju_negQuat(rel_quat, data.xquat[em_pad_id])

    assert abs(np.linalg.norm(rel_quat) - 1.0) < 1e-6, \
        f"rel_quat not unit: norm={np.linalg.norm(rel_quat)}"

    # MuJoCo 3 weld eq_data layout (11 values per constraint):
    #   [0:3] anchor (body1 frame) | [3:6] relpos | [6:10] relquat [w,x,y,z] | [10] torquescale
    model.eq_data[weld_id, 3:6]  = rel_pos
    model.eq_data[weld_id, 6:10] = rel_quat

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
