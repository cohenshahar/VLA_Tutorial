"""
scene/em_controller.py
──────────────────────────────────────────────────────────────────────────
Phase 6 — Electromagnetic attachment controller (Tasks 6.5 and 6.6).
Phase 10, Task 10.6 — Shape-agnostic proximity check added.

Design: "magnetic connection already solved."
  em_activate() does not simulate magnetic attraction physics.
  When preconditions are met it snaps the object to the ideal attached
  geometry — top site flush with EM face — and locks the weld.

API
───
  em_can_activate(data, model, threshold_m=0.02) → bool
      Shape-agnostic: reads the proximity sensor (sensor_proximity).
      True when sensor reads < threshold_m AND (for the box) orientation OK.
      Works for both metal_box and metal_sphere without modification.

  em_activate(model, data, threshold_m=0.02, object_name="metal_box") → bool
      Checks proximity; if OK, snaps the named object to the ideal
      attached pose and activates the em_weld constraint.
      Supports "metal_box" (half=0.05) and "metal_sphere" (half=0.04).
      Returns True if activated, False if preconditions failed.

  em_deactivate(model, data) → True
      Unconditionally disables the weld.

Backward compatibility
──────────────────────
  All existing call sites that use em_activate(model, data, threshold_m=...)
  continue to work (defaults to metal_box).
"""

import numpy as np
import mujoco

# Site offset of em_contact_site in em_pad frame
_EM_SITE_LOCAL = np.array([0.01, 0.0, 0.0])

# Per-object top-site local offset and freejoint name
_OBJECT_CFG = {
    "metal_box": {
        "top_local":  np.array([0.0, 0.0, 0.05]),   # box_top_site offset in body frame
        "freejoint":  "box_freejoint",
        "upright_check": True,                        # require ~upright orientation
    },
    "metal_sphere": {
        "top_local":  np.array([0.0, 0.0, 0.04]),   # sphere_top_site offset
        "freejoint":  "sphere_freejoint",
        "upright_check": False,                       # sphere has no preferred orientation
    },
}

_UPRIGHT_QUAT = np.array([1.0, 0.0, 0.0, 0.0])   # identity [w, x, y, z]


# ── Private helpers ───────────────────────────────────────────────────────────

def _get_proximity(model: mujoco.MjModel,
                   data: mujoco.MjData) -> float:
    """
    Read the proximity sensor (sensor_proximity) value.

    Returns float distance in metres. Returns 1.0 (max range) if sensor
    is not found in the model (safe fallback — prevents false activations).
    """
    prox_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                  "sensor_proximity")
    if prox_sid < 0:
        return 1.0   # sensor not present — safe fallback
    return float(data.sensordata[model.sensor_adr[prox_sid]])


# ── Public API ────────────────────────────────────────────────────────────────

def em_can_activate(data: mujoco.MjData,
                    model: mujoco.MjModel,
                    threshold_m: float = 0.02) -> bool:
    """
    Shape-agnostic proximity check using the rangefinder sensor.

    Returns True when:
      1. sensor_proximity reads < threshold_m  (EM face is close to an object).
      2. The metal_box is approximately upright (if present in model) —
         checked only to prevent grasping a tipped box.
         Sphere has no orientation requirement.

    This replaces the Phase-6 site-distance check and works for any
    object shape without modification.

    Parameters
    ----------
    data        : mujoco.MjData
    model       : mujoco.MjModel
    threshold_m : distance threshold (m); default 0.02 m.
    """
    # 1. Proximity sensor check (shape-agnostic)
    proximity = _get_proximity(model, data)
    if proximity >= threshold_m:
        return False

    # 2. Box orientation check (only if metal_box is in the scene)
    box_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "metal_box")
    if box_bid >= 0:
        box_z_world = data.xmat[box_bid].reshape(3, 3)[:, 2]
        if box_z_world[2] < np.cos(np.radians(20)):
            return False   # box is too tilted — refuse grasp

    return True


def em_activate(model: mujoco.MjModel,
                data: mujoco.MjData,
                threshold_m: float = 0.02,
                object_name: str = "metal_box") -> bool:
    """
    Activate the em_weld constraint if proximity precondition is met.

    Snaps the named object's top site flush with the EM contact site,
    makes the object world-upright (for box) or keeps orientation
    (for sphere, where orientation is unconstrained), then activates.

    Parameters
    ----------
    model       : mujoco.MjModel
    data        : mujoco.MjData
    threshold_m : proximity threshold (m); same as em_can_activate.
    object_name : "metal_box" (default) or "metal_sphere".

    Returns
    -------
    True if activated, False if preconditions failed or object not found.
    """
    if object_name not in _OBJECT_CFG:
        raise ValueError(
            f"Unknown object '{object_name}'. "
            f"Supported: {list(_OBJECT_CFG.keys())}")

    if not em_can_activate(data, model, threshold_m=threshold_m):
        return False

    cfg = _OBJECT_CFG[object_name]
    top_local = cfg["top_local"]    # object top-site offset in body frame
    joint_name = cfg["freejoint"]

    # Resolve constraint and body IDs
    weld_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
    em_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     "em_pad")
    obj_bid   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,     object_name)
    obj_jnt   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,    joint_name)

    if weld_id < 0:
        raise RuntimeError("em_weld equality constraint not found in model.")
    if em_pad_id < 0:
        raise RuntimeError("em_pad body not found in model.")
    if obj_bid < 0:
        raise RuntimeError(f"Body '{object_name}' not found in model.")
    if obj_jnt < 0:
        raise RuntimeError(f"Joint '{joint_name}' not found in model.")

    mat_em = data.xmat[em_pad_id].reshape(3, 3)
    p_em   = data.xpos[em_pad_id].copy()

    # World position of em_contact_site
    p_site = p_em + mat_em @ _EM_SITE_LOCAL

    # Ideal object centre: top site coincides with EM contact site.
    # top_local is the site offset in body frame — when body is world-upright,
    # this is simply [0, 0, half_height] in world frame.
    p_obj_ideal = p_site - top_local

    # Snap object to ideal pose, zero velocity
    qpa = model.jnt_qposadr[obj_jnt]
    doa = model.jnt_dofadr[obj_jnt]
    data.qpos[qpa:qpa + 3] = p_obj_ideal
    data.qpos[qpa + 3:qpa + 7] = _UPRIGHT_QUAT   # world-upright for both box and sphere
    data.qvel[doa:doa + 6] = 0.0
    mujoco.mj_forward(model, data)

    # Relative pose in em_pad frame for weld constraint:
    #   rel_pos  = R_em.T @ (p_obj_ideal − p_em)  − R_em.T @ top_local  (via EM site)
    rel_pos  = _EM_SITE_LOCAL - mat_em.T @ top_local
    rel_quat = np.zeros(4)
    mujoco.mju_negQuat(rel_quat, data.xquat[em_pad_id])

    assert abs(np.linalg.norm(rel_quat) - 1.0) < 1e-6, \
        f"rel_quat not unit: norm={np.linalg.norm(rel_quat)}"

    # MuJoCo 3 weld eq_data layout (11 values):
    #   [0:3] anchor | [3:6] relpos | [6:10] relquat [w,x,y,z] | [10] torquescale
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
