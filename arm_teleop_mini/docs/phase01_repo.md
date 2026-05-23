# Phase 01 — Repo + colcon workspace

**Goal**: A buildable empty ROS2 package that `colcon build` accepts with no errors.

**Concepts introduced**:
- `ament_python` package layout
- `package.xml`, `setup.py`, `setup.cfg`
- colcon workspace conventions

**Deliverables**:
- `arm_teleop/package.xml`
- `arm_teleop/setup.py`
- `arm_teleop/setup.cfg`
- `arm_teleop/resource/arm_teleop`

**Smoke test**:
- Command: `colcon build --packages-select arm_teleop`
- Expected: build finishes with `1 package finished`

**Notes** (filled in as we go):
