---
description: "Validate a MuJoCo MJCF or URDF file. Use when: checking a model file loads correctly, verifying joint count, joint names, joint limits, or rendering a home-pose PNG."
name: "Validate MJCF / URDF"
argument-hint: "Path to .xml or .urdf file, relative to VLATraining/sim/"
agent: "agent"
tools: ["read_file", "run_in_terminal", "create_file"]
---

Validate the MuJoCo model file: **$MODEL_PATH**

Run the following checks in order. Use the venv at `phase4_env/`:
```bash
source /home/shahar/Desktop/phase4/phase4_env/bin/activate
cd /home/shahar/Desktop/phase4/VLATraining/sim
```

## Check 1 — Load without errors

```python
import mujoco
model = mujoco.MjModel.from_xml_path("$MODEL_PATH")
print("nq =", model.nq)
print("nv =", model.nv)
print("nbody =", model.nbody)
```

Expected: no exception, `nq` printed.

## Check 2 — Joint inventory

```python
import mujoco, numpy as np
model = mujoco.MjModel.from_xml_path("$MODEL_PATH")
for i in range(model.njnt):
    j = model.joint(i)
    lo, hi = np.degrees(model.jnt_range[i])
    print(f"  {j.name:20s}  type={j.type}  limits=[{lo:.1f}°, {hi:.1f}°]")
```

For the KUKA KR 6 R900 sixx, verify these limits (±2° tolerance):

| Joint | Lower | Upper |
|---|---|---|
| A1 | −170° | +170° |
| A2 | −190° | +45° |
| A3 | −120° | +156° |
| A4 | −185° | +185° |
| A5 | −120° | +120° |
| A6 | −350° | +350° |

Report PASS/FAIL per joint.

## Check 3 — Render home-pose PNG

```python
import mujoco, numpy as np
from PIL import Image

model = mujoco.MjModel.from_xml_path("$MODEL_PATH")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

renderer = mujoco.Renderer(model, height=720, width=1280)
renderer.update_scene(data)
pixels = renderer.render()
Image.fromarray(pixels).save("outputs/validate_home_pose.png")
print("Saved: outputs/validate_home_pose.png")
```

Confirm `outputs/validate_home_pose.png` is created and non-zero in size.

## Check 4 — Physics stability (100 steps, no NaN)

```python
import mujoco, numpy as np

model = mujoco.MjModel.from_xml_path("$MODEL_PATH")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

for _ in range(100):
    mujoco.mj_step(model, data)

assert not np.any(np.isnan(data.qpos)), "NaN in qpos after 100 steps!"
print("Physics stable. Final qpos:", np.round(data.qpos, 4))
```

## Summary

Report a table:

| Check | Result | Notes |
|---|---|---|
| Load without errors | PASS/FAIL | |
| nq == 6 (KUKA) | PASS/FAIL | actual value |
| All 6 joint limits correct | PASS/FAIL | list any failures |
| Home-pose PNG saved | PASS/FAIL | file path |
| Physics stable (no NaN) | PASS/FAIL | |
