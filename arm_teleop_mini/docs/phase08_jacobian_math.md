# Phase 08 — Jacobian IK math (notebook)

**Goal**: Damped least-squares pseudoinverse is derived on paper and verified numerically in the notebook.

**Concepts introduced**:
- Geometric Jacobian via `mujoco.mj_jacBody`
- Condition number and singularity
- DLS formula: `q_dot = J^T @ inv(J J^T + λ² I) @ x_dot`
- `kinematics.py` stubs filled in

**Deliverables**:
- `notebooks/08_jacobian_ik_math.ipynb` — all code cells implemented and runnable
- `arm_teleop/kinematics.py` — `compute_jacobian`, `damped_least_squares_ik` implemented

**Smoke test**:
- Command: run notebook end-to-end (Jupyter or `nbconvert`)
- Expected: all cells execute without error; final cell prints a valid `q_dot` vector

**Notes** (filled in as we go):
