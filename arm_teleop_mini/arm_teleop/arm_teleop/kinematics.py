"""
kinematics — Jacobian and damped-least-squares IK.

Introduced in Phase 8 (math notebook) and used by cartesian_controller_node (Phase 9).

This module is the *only* place where IK math lives. Keep it pure: no ROS imports,
no MuJoCo stepping. Take in numpy arrays and MuJoCo model+data, return numpy arrays.

Math reference (paper-level):
  Forward velocity kinematics:   x_dot = J(q) @ q_dot
  Naive inverse:                 q_dot = inv(J) @ x_dot     (blows up near singularities)
  Damped least squares:          q_dot = J^T @ inv(J J^T + lambda^2 * I) @ x_dot
"""

# import numpy as np    # TODO Phase 8
# import mujoco         # TODO Phase 8


def compute_jacobian(model, data, body_name: str = "tool0"):
    """
    Compute the 6x6 geometric Jacobian of `body_name` w.r.t. the 6 joints.

    Args:
        model: mujoco.MjModel
        data:  mujoco.MjData (must be up-to-date — call mj_forward first)
        body_name: name of the end-effector body in the MJCF

    Returns:
        J: numpy array shape (6, 6). Rows 0..2 = linear, rows 3..5 = angular.
    """
    # TODO Phase 8:
    #   - body_id = mujoco.mj_name2id(model, mjOBJ_BODY, body_name)
    #   - jacp = np.zeros((3, model.nv));  jacr = np.zeros((3, model.nv))
    #   - mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
    #   - J = np.vstack([jacp[:, :6], jacr[:, :6]])
    #   - return J
    raise NotImplementedError("Phase 8")


def damped_least_squares_ik(J, x_dot, lambda_sq: float = 0.01):
    """
    Solve q_dot from desired x_dot using damped least-squares pseudoinverse.

    Math:
        q_dot = J^T @ inv(J @ J^T + lambda^2 * I) @ x_dot

    Args:
        J: (6, 6) Jacobian
        x_dot: (6,) desired EE velocity [vx, vy, vz, wx, wy, wz]
        lambda_sq: damping factor (squared). Larger = safer near singularities,
                   but slower convergence. Default 0.01.

    Returns:
        q_dot: (6,) joint velocities.
    """
    # TODO Phase 8:
    #   I6 = np.eye(6)
    #   A  = J @ J.T + lambda_sq * I6
    #   q_dot = J.T @ np.linalg.solve(A, x_dot)
    #   return q_dot
    raise NotImplementedError("Phase 8")


def quat_to_rotation_matrix(quat):
    """
    Convert a MuJoCo quaternion [w, x, y, z] to a 3x3 rotation matrix.

    Used in Phase 10 to rotate a tool-frame Twist into base frame.
    """
    # TODO Phase 10
    raise NotImplementedError("Phase 10")
