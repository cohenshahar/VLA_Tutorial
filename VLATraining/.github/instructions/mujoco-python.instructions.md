---
applyTo: "**/VLATraining/sim/**/*.py"
description: "MuJoCo Python API conventions for the VLA simulation project."
---

## MuJoCo Python API conventions

### Model and data loading
```python
import mujoco
model = mujoco.MjModel.from_xml_path("scene/world.xml")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)
```

### Named lookups — always prefer by name over hard-coded index
```python
body_id  = model.body("box").id
site_id  = model.site("em_contact_site").id
joint_id = model.joint("A1").id
sensor_id = model.sensor("proximity").id
eq_id    = model.equality("em_weld").id
```

### Reading sensor values
```python
adr = model.sensor_adr[sensor_id]
dim = model.sensor_dim[sensor_id]
value = data.sensordata[adr:adr + dim]   # numpy slice
```

### Actuator control (joint position targets)
```python
# Find actuator index by name
act_id = model.actuator("A1_pos").id
data.ctrl[act_id] = target_rad           # radians
```

### EM weld constraint toggle
```python
em_id = model.equality("em_weld").id
model.eq_active[em_id] = True    # attach
model.eq_active[em_id] = False   # release
```
Pre-condition before activating: `data.sensordata[proximity_adr] < 0.02`.

### Camera rendering (offscreen, no display required)
```python
# IMPORTANT: set framebuffer size on model BEFORE creating Renderer.
# The <mujoco><visual><global offwidth/offheight> URDF tag is NOT read at runtime.
model.vis.global_.offwidth = 1280
model.vis.global_.offheight = 720
renderer = mujoco.Renderer(model, height=720, width=1280)
renderer.update_scene(data, camera="wrist_cam")
pixels = renderer.render()   # numpy uint8 [H, W, 3]
```
For saving: `from PIL import Image; Image.fromarray(pixels).save("outputs/name.png")`

### Physics step loop pattern
```python
while data.time < duration:
    mujoco.mj_step(model, data)
```
Never use `time.sleep()` inside the physics loop — it decouples wall time from sim time.

### Passive viewer (for interactive debugging only)
```python
with mujoco.viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
```

### Body position / orientation
```python
pos = data.xpos[model.body("box").id]        # world position [3]
quat = data.xquat[model.body("box").id]      # world quaternion [4]
```

### Applying external forces (test impulses only)
```python
data.xfrc_applied[body_id] = [fx, fy, fz, tx, ty, tz]
# Reset after use:
data.xfrc_applied[body_id] = 0
```

### Output paths
All renders → `sim/outputs/<name>.png`
All videos → `sim/outputs/<name>.mp4`
All CSVs  → `sim/outputs/<run_id>_<component>.csv`

### Anti-patterns to avoid
- Do NOT use deprecated `mj.mjr_render` / `mj.mjr_readPixels` directly — use `mujoco.Renderer`.
- Do NOT hard-code sensor indices as integers — use `model.sensor(name).id`.
- Do NOT call `input()` or any interactive prompt.
- Do NOT use joint index integers to set `data.ctrl` without first confirming the actuator order matches.
