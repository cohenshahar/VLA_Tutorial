"""
Kinematic solvers for the KUKA KR6 R900 sixx arm.

Public API:
    ik_jacobian(model, data, target_xyz, ...) -> (qpos, eef_pos)
"""
import numpy as np
import mujoco


def ik_jacobian(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target_xyz,
    eef_body: str = "link_6",
    max_iter: int = 500,
    tol: float = 0.005,
    step: float = 0.5,
) -> tuple:
    """Drive the end-effector to target_xyz using the Jacobian pseudoinverse.

    Pure kinematic solver: writes directly to data.qpos[:6] until convergence,
    then syncs data.ctrl and runs 1000 physics steps for gravity settle.
    Error is measured at the kinematic solution, before the physics settle.

    Returns (final_qpos_6, eef_pos_at_convergence).
    """
    target = np.array(target_xyz, dtype=float)
    eef_id = model.body(eef_body).id
    jac_p  = np.zeros((3, model.nv))

    for i in range(max_iter):
        mujoco.mj_forward(model, data)
        pos_err  = target - data.xpos[eef_id]
        err_norm = np.linalg.norm(pos_err)

        if err_norm < tol:
            print(f"  IK converged in {i+1} iterations  (err={err_norm*1000:.2f} mm)")
            break

        mujoco.mj_jacBody(model, data, jac_p, None, eef_id)
        jac_arm = jac_p[:, :6]  # first 6 columns = arm DOFs

        dq, _, _, _ = np.linalg.lstsq(jac_arm, pos_err, rcond=None)
        data.qpos[:6] += step * dq

        # Clamp to joint limits after each step
        for j in range(6):
            jid    = model.joint(f"joint_a{j+1}").id
            lo, hi = model.jnt_range[jid]
            data.qpos[j] = float(np.clip(data.qpos[j], lo, hi))
    else:
        final_err = np.linalg.norm(target - data.xpos[eef_id]) * 1000
        print(f"  IK did NOT converge (final err={final_err:.1f} mm)")

    mujoco.mj_forward(model, data)
    eef_pos = data.xpos[eef_id].copy()

    # Sync controllers and settle with physics (for visualization only)
    data.ctrl[:6] = data.qpos[:6].copy()
    for _ in range(1000):
        mujoco.mj_step(model, data)

    return data.qpos[:6].copy(), eef_pos
