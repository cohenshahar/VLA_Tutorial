"""
scene/randomize_scene.py
─────────────────────────────────────────────────────────────────────────
Phase 5 — Randomize box and target zone positions on the table.

Call randomize_scene(model, data) ONCE before the first mj_step.
It modifies:
  - data.qpos[box_qpos_start : box_qpos_start+7]  — box freejoint (pos + quat)
  - model.body_pos[target_zone_id]                 — target zone body position

Then calls mj_forward() to propagate kinematics.

Table workspace used for randomization (arm's reachable area):
  World coordinates, table surface Z = 0.85 m.
  Arm base at (0, -0.35, 0.85) — back edge of table.
  Safe reachable zone: X ∈ [-0.25, 0.25], Y ∈ [0.00, 0.25]

Constraints:
  - Box is placed upright (identity quaternion, random Z rotation only).
  - Box and target zone start at least MIN_SEPARATION metres apart.
  - Box Z = 0.92 (7 cm above table surface — falls and lands cleanly).
  - Target zone Z = 0.87 (orange pad sits on table surface, 0.02 m half-height → top at 0.89).

Usage
─────
  from scene.randomize_scene import randomize_scene
  model = mujoco.MjModel.from_xml_path(WORLD)
  data  = mujoco.MjData(model)
  randomize_scene(model, data, seed=42)
  # now run mj_step normally
"""

import math
import numpy as np
import mujoco

# ── Workspace bounds (table reachable area) ────────────────────────────
X_MIN, X_MAX = -0.25,  0.25
Y_MIN, Y_MAX =  0.00,  0.25

# Height constants
BOX_DROP_Z   = 0.92   # drop height → box falls ~7 cm and lands flat
TARGET_Z     = 0.87   # orange pad surface: base at 0.85, half-height 0.01, centre at 0.86 → 0.87 keeps it flush

MIN_SEPARATION = 0.20  # minimum horizontal distance between box centre and target centre


def randomize_scene(model: mujoco.MjModel,
                    data: mujoco.MjData,
                    seed: int | None = None) -> dict:
    """
    Randomize box and target zone positions.

    Returns
    -------
    dict with keys:
        'box_pos'    : np.ndarray (3,) — world XYZ of box centre
        'target_pos' : np.ndarray (3,) — world XYZ of target zone centre
        'box_yaw_deg': float — random Z rotation applied to box (degrees)
    """
    rng = np.random.default_rng(seed)

    # ── Box position ──────────────────────────────────────────────────
    box_xy = rng.uniform([X_MIN, Y_MIN], [X_MAX, Y_MAX])
    box_pos = np.array([box_xy[0], box_xy[1], BOX_DROP_Z])

    # ── Target zone position (must be MIN_SEPARATION from box) ────────
    for _ in range(1000):
        tgt_xy = rng.uniform([X_MIN, Y_MIN], [X_MAX, Y_MAX])
        dist   = float(np.linalg.norm(tgt_xy - box_xy))
        if dist >= MIN_SEPARATION:
            break
    else:
        # Fallback: force target to opposite quadrant
        tgt_xy = np.array([-box_xy[0] * 0.8, box_xy[1] + 0.22])
        tgt_xy = np.clip(tgt_xy, [X_MIN, Y_MIN], [X_MAX, Y_MAX])

    target_pos = np.array([tgt_xy[0], tgt_xy[1], TARGET_Z])

    # ── Box orientation: upright with random yaw ─────────────────────
    yaw = rng.uniform(0, 2 * math.pi)
    # Quaternion for Z-axis rotation: [w, x, y, z]
    box_quat = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])

    # ── Apply box freejoint (7 values: pos[3] + quat[4]) ─────────────
    box_jnt_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_freejoint")
    if box_jnt_id < 0:
        raise RuntimeError("box_freejoint not found in model.")
    box_qpos_adr = model.jnt_qposadr[box_jnt_id]

    data.qpos[box_qpos_adr:box_qpos_adr + 3] = box_pos
    data.qpos[box_qpos_adr + 3:box_qpos_adr + 7] = box_quat

    # Zero out box velocity
    box_dof_adr = model.jnt_dofadr[box_jnt_id]
    data.qvel[box_dof_adr:box_dof_adr + 6] = 0.0

    # ── Apply target zone body position ───────────────────────────────
    target_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target_zone")
    if target_body_id < 0:
        raise RuntimeError("target_zone body not found in model.")
    model.body_pos[target_body_id] = target_pos

    # ── Propagate kinematics ──────────────────────────────────────────
    mujoco.mj_forward(model, data)

    return {
        "box_pos":     box_pos.copy(),
        "target_pos":  target_pos.copy(),
        "box_yaw_deg": math.degrees(yaw),
    }


if __name__ == "__main__":
    import os

    SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    WORLD   = os.path.join(SIM_DIR, "scene", "world.xml")

    model = mujoco.MjModel.from_xml_path(WORLD)
    data  = mujoco.MjData(model)

    info = randomize_scene(model, data, seed=42)
    print("Randomization result:")
    print(f"  Box start    : {info['box_pos']}")
    print(f"  Target centre: {info['target_pos']}")
    print(f"  Box yaw      : {info['box_yaw_deg']:.1f}°")
    separation = float(np.linalg.norm(info['box_pos'][:2] - info['target_pos'][:2]))
    print(f"  XY separation: {separation:.3f} m  (min {MIN_SEPARATION} m)")
    assert separation >= MIN_SEPARATION, "FAIL: separation too small"
    print("  randomize_scene PASS ✓")
