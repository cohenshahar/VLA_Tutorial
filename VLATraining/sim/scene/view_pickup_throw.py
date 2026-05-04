"""
scene/view_pickup_throw.py  —  live viewer: step-by-step pick approach, loops forever
══════════════════════════════════════════════════════════════════════════════════
Named positions per cycle:
  P0  RANDOM   — random safe arm pose, random box on table (initial state)
  P1  READY    — work position: arm raised, EM pad facing straight down
  P2  HUB      — intermediate: A1 snapped to 0° (front) or ±180° (back)
  P3  ALIGNED  — A1 rotated to face the box on the same radial line
  (more to come: P4 ABOVE_BOX, P5 SNAP, P6 GRAB, P7 LIFT, P8 THROW)
Close the window to stop.
"""

import math
import pathlib
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_can_activate, em_deactivate

# ── constants ─────────────────────────────────────────────────────────────────
WORLD_XML = str(pathlib.Path(__file__).parent / "world.xml")
TABLE_Z   = 0.85
BOX_HALF  = 0.05
ARM_BASE  = np.array([0.0, -0.3])
EXCL_R    = 0.22
MAX_REACH = 0.62
MOVE_SECS    = 3.0    # seconds for each lerp move
HOLD_SECS    = 1.5    # seconds to hold at each named position
RENDER_EVERY = 8      # step multiple at which we call viewer.sync()

_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}
_ACT_NAMES   = [f"act_a{i}" for i in range(1, 7)]
_JOINT_NAMES = [f"joint_a{i}" for i in range(1, 7)]

# Narrower safe ranges to avoid self-collision.
# Full URDF limits are used for A1/A4/A6; A2/A3/A5 are constrained.
_SAFE_RANGES = [
    (-2.97,  2.97),   # A1 — base rotation, full range fine
    (-1.40,  0.50),   # A2 — shoulder: avoid extreme backward lean
    (-0.50,  2.00),   # A3 — elbow: keep arm reasonably extended
    (-3.14,  3.14),   # A4 — wrist rotation, large range ok
    (-1.50,  1.50),   # A5 — wrist pitch, moderate
    (-3.14,  3.14),   # A6 — end-effector spin, fine
]


def _random_box(rng):
    for _ in range(2000):
        bx = rng.uniform(-0.60, 0.60)
        by = rng.uniform(-0.25, 0.28)
        r  = float(np.linalg.norm([bx - ARM_BASE[0], by - ARM_BASE[1]]))
        if EXCL_R < r <= MAX_REACH:
            return np.array([bx, by, TABLE_Z + BOX_HALF])
    raise RuntimeError("Cannot sample a valid box position.")


# ── IK solver (DLS, runs on a scratch MjData — never touches live sim) ────────
APPROACH_Z = TABLE_Z + 0.40   # P4: EE clearance height above table
LIFT_Z     = TABLE_Z + 0.40   # P7: lift height (same clearance, box attached)
# Drop position: x=0.65 puts the box just outside the table edge (table half-width=0.6m)
DROP_XY    = np.array([0.65, -0.3])  # outside table, y=ARM_BASE[1] for easy reach
# P5 IK target = box_top + SNAP_GAP.  The gap guarantees no penetration even
# when the PD controller overshoots.  Measured overshoot: ~3 cm.
SNAP_GAP   = 0.055            # IK target sits 5.5 cm above box top → arm settles ~2-3 cm above
# Seed for P5 IK: joints bent to bring EE near snap height (from em_pickup_test SNAP_POSE).
# A1 overridden at runtime with a1_target; A2/A3/A5 matched to SNAP_POSE.
SNAP_SEED_Q = np.array([0.0, -0.96, 1.57, 0.0, 0.96, 0.0])

# R_DOWN: site x-axis = world +Z  (EM face pointing down)
R_DOWN = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])

def _solve_ik(model, qpas, dofas, ranges, site_id, seed_q, tgt_pos):
    """DLS IK seeded from seed_q. Returns solved joint array."""
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
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)
    DT = model.opt.timestep

    jids    = [model.joint(n).id         for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j]      for j in jids]
    dofas   = [model.jnt_dofadr[j]       for j in jids]
    ranges  = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id      for n in _ACT_NAMES]
    site_id     = model.site("em_contact_site").id
    box_site_id = model.site("box_top_site").id
    box_jid = model.joint("box_freejoint").id
    bqpa    = model.jnt_qposadr[box_jid]

    # Arm body ids to check for table clearance (link_1..link_6, em_pad)
    arm_body_ids = [model.body(n).id for n in
                    ["link_1","link_2","link_3","link_4","link_5","link_6","em_pad"]]

    def arm_above_table(q):
        """Return True if every arm body is at or above TABLE_Z after forward kinematics."""
        d_tmp = mujoco.MjData(model)
        for qi, v in zip(qpas, q):
            d_tmp.qpos[qi] = v
        mujoco.mj_forward(model, d_tmp)
        return all(d_tmp.xpos[bid, 2] >= TABLE_Z for bid in arm_body_ids)

    def random_arm_q():
        for _ in range(500):
            q = np.array([rng.uniform(lo, hi) for lo, hi in _SAFE_RANGES])
            if arm_above_table(q):
                return q
        # Fallback: HOME pose is always safe
        return np.zeros(6)

    def smooth(t):
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

    # READY pose: site x-axis ≈ (0,0,+1) matching R_DOWN — EM face points straight down.
    # A2=-1.05, A3=1.65, A5=0.96 → EE 49 cm above table, all bodies clear.
    READY_Q = np.array([0.0, -1.05, 1.65, 0.0, 0.96, 0.0])

    rng   = np.random.default_rng()
    cycle = 0

    move_steps = max(1, int(MOVE_SECS / DT))
    hold_steps = max(1, int(HOLD_SECS / DT))

    print("Pick approach viewer — close window to stop\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.2, 0.0, 1.0]
        viewer.cam.distance  = 2.2
        viewer.cam.azimuth   = 140
        viewer.cam.elevation = -22

        step_n = 0

        # ── shared motion helpers ─────────────────────────────────────────────
        def _lerp_phase(q_from, q_to):
            t0 = time.time()
            for step_i in range(move_steps):
                if not viewer.is_running():
                    return
                frac = smooth(step_i / move_steps)
                for ai, v0, v1 in zip(act_ids, q_from, q_to):
                    data.ctrl[ai] = v0 + frac * (v1 - v0)
                mujoco.mj_step(model, data)
                nonlocal step_n
                step_n += 1
                if step_n % RENDER_EVERY == 0:
                    viewer.sync()
                    lag = (t0 + step_i * DT) - time.time()
                    if lag > 0:
                        time.sleep(lag)

        def _hold_phase(q):
            t0 = time.time()
            for step_i in range(hold_steps):
                if not viewer.is_running():
                    return
                for ai, v in zip(act_ids, q):
                    data.ctrl[ai] = v
                mujoco.mj_step(model, data)
                nonlocal step_n
                step_n += 1
                if step_n % RENDER_EVERY == 0:
                    viewer.sync()
                    lag = (t0 + step_i * DT) - time.time()
                    if lag > 0:
                        time.sleep(lag)

        while viewer.is_running():
            # ── RESET: new random arm pose + box position ─────────────────────
            mujoco.mj_resetData(model, data)
            apply_gains(model, _GAINS)
            cycle += 1

            # Random joint angles within safe (collision-free) ranges,
            # guaranteed to keep every arm body above the table
            arm_q = random_arm_q()
            for qi, q in zip(qpas, arm_q):
                data.qpos[qi] = q
            for ai, q in zip(act_ids, arm_q):
                data.ctrl[ai] = q

            # Random box on table
            box_pos = _random_box(rng)
            data.qpos[bqpa:bqpa + 7] = [*box_pos, 1., 0., 0., 0.]

            mujoco.mj_forward(model, data)

            # ── Compute hub / aligned / above positions ───────────────────────
            dx = box_pos[0] - ARM_BASE[0]
            dy = box_pos[1] - ARM_BASE[1]
            a1_lo, a1_hi = model.jnt_range[jids[0]]
            a1_target = float(np.clip(math.atan2(-dy, dx), a1_lo, a1_hi))

            if abs(a1_target) <= math.pi / 2:
                hub_a1   = 0.0
                hub_name = "P1 READY    A1=0°"
            else:
                hub_a1   = float(np.clip(math.copysign(math.pi, a1_target), a1_lo, a1_hi))
                hub_name = f"P2 HUB      A1={math.degrees(hub_a1):+.0f}°"

            hub_q     = READY_Q.copy(); hub_q[0] = hub_a1
            aligned_q = READY_Q.copy(); aligned_q[0] = a1_target

            # ── Pre-solve IK for P4 ABOVE_BOX only (offline) ─────────────────
            print(f"\nCycle {cycle:3d}  Solving P4 IK...", end=" ", flush=True)
            above_tgt = np.array([box_pos[0], box_pos[1], APPROACH_Z])
            above_q   = _solve_ik(model, qpas, dofas, ranges, site_id,
                                  aligned_q, above_tgt)
            snap_z    = box_pos[2] + BOX_HALF + SNAP_GAP  # box top + safety gap
            snap_tgt  = np.array([box_pos[0], box_pos[1], snap_z])
            print("done")

            # ── P0 → hub ─────────────────────────────────────────────────────
            print(f"  {hub_name}  (box at {math.degrees(a1_target):+.1f}°)")
            _lerp_phase(arm_q, hub_q)
            _hold_phase(hub_q)

            # ── hub → P3 ALIGNED ─────────────────────────────────────────────
            print(f"  P3 ALIGNED  A1={math.degrees(a1_target):+.1f}°")
            _lerp_phase(hub_q, aligned_q)
            _hold_phase(aligned_q)

            box_top = box_pos[2] + BOX_HALF   # 0.95 m

            # ── P3 → P4 ABOVE_BOX ────────────────────────────────────────────
            print(f"  P4 ABOVE_BOX  IK_tgt_z={APPROACH_Z:.3f}")
            _lerp_phase(aligned_q, above_q)
            _hold_phase(above_q)
            actual_z = data.site_xpos[site_id][2]
            print(f"             actual EE_z={actual_z:.3f}  (box_top={box_top:.3f})")

            # ── Re-solve P5 IK from ACTUAL joint positions after P4 hold ─────
            # Seeding from real state gives IK a much closer starting config,
            # so snap_q will accurately place the site near (box_x, box_y, snap_z).
            print(f"  P5 SNAP       Re-solving IK from actual...", end=" ", flush=True)
            actual_q = np.array([data.qpos[qi] for qi in qpas])
            snap_q   = _solve_ik(model, qpas, dofas, ranges, site_id,
                                 actual_q, snap_tgt)
            print(f"done  IK_tgt_z={snap_z:.3f}  (box_top+{SNAP_GAP:.3f})")
            _lerp_phase(above_q, snap_q)
            _hold_phase(snap_q)
            actual_z  = data.site_xpos[site_id][2]
            above     = actual_z - box_top
            ee_pos    = data.site_xpos[site_id].copy()
            bt_pos    = data.site_xpos[box_site_id].copy()
            dist3d    = float(np.linalg.norm(ee_pos - bt_pos))
            xy_err    = float(np.linalg.norm(ee_pos[:2] - bt_pos[:2]))
            status    = "OK" if above > 0 else "WARN: inside box!"
            print(f"             actual EE_z={actual_z:.3f}  above_box={above:+.4f}m  dist3d={dist3d:.4f}m  xy_err={xy_err:.4f}m  [{status}]")

            # ── P5 → P6 GRAB ─────────────────────────────────────────────────
            # Hold snap_q and call em_activate every step — the arm oscillates
            # slightly, so it will pass within the 2 cm threshold.
            print(f"  P6 GRAB       trying...", end=" ", flush=True)
            grabbed = False
            t0 = time.time()
            for step_i in range(hold_steps):
                if not viewer.is_running():
                    break
                for ai, v in zip(act_ids, snap_q):
                    data.ctrl[ai] = v
                mujoco.mj_step(model, data)
                step_n += 1
                if step_n % RENDER_EVERY == 0:
                    viewer.sync()
                    lag = (t0 + step_i * DT) - time.time()
                    if lag > 0:
                        time.sleep(lag)
                if not grabbed and em_activate(model, data, threshold_m=0.06):
                    grabbed = True
            print(f"{'OK — box attached' if grabbed else 'FAILED'}")

            # ── P6 → P7 LIFT ─────────────────────────────────────────────────
            # Raise EE (+ attached box) straight up to LIFT_Z.
            print(f"  P7 LIFT       Re-solving IK...", end=" ", flush=True)
            actual_q = np.array([data.qpos[qi] for qi in qpas])
            lift_tgt = np.array([box_pos[0], box_pos[1], LIFT_Z])
            lift_q   = _solve_ik(model, qpas, dofas, ranges, site_id, actual_q, lift_tgt)
            print(f"done")
            _lerp_phase(snap_q, lift_q)
            _hold_phase(lift_q)
            box_z = data.qpos[bqpa + 2]
            print(f"             box_z={box_z:.3f}  ({'lifted' if box_z > TABLE_Z + 0.05 else 'WARN: not lifted'})")

            # ── P7 → P8 CARRY ────────────────────────────────────────────────
            # Swing arm so box is over DROP_XY — outside the table edge.
            print(f"  P8 CARRY      Re-solving IK for drop zone...", end=" ", flush=True)
            actual_q  = np.array([data.qpos[qi] for qi in qpas])
            carry_tgt = np.array([DROP_XY[0], DROP_XY[1], LIFT_Z])
            carry_q   = _solve_ik(model, qpas, dofas, ranges, site_id, actual_q, carry_tgt)
            print(f"done  drop=({DROP_XY[0]:.2f},{DROP_XY[1]:.2f})")
            _lerp_phase(lift_q, carry_q)
            _hold_phase(carry_q)
            ee_xy = data.site_xpos[site_id][:2]
            print(f"             EE_xy=({ee_xy[0]:+.3f},{ee_xy[1]:+.3f})")

            # ── P8 → P9 DROP ─────────────────────────────────────────────────
            # Deactivate EM — box falls outside the table.
            em_deactivate(model, data)
            print(f"  P9 DROP       EM released — box falling...")
            _hold_phase(carry_q)
            box_z = data.qpos[bqpa + 2]
            box_xy = data.qpos[bqpa:bqpa + 2]
            print(f"             box landed at ({box_xy[0]:+.3f},{box_xy[1]:+.3f},{box_z:.3f})")


if __name__ == "__main__":
    main()
