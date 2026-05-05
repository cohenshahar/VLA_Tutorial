---
name: scene-snapshot
description: "Render and save a visual snapshot of the current MuJoCo scene. Use when: show me the scene, what does the scene look like, render cameras, preview, take a snapshot, view current state, check camera views, save renders, what do the cameras see, screenshot the sim."
argument-hint: "Optional: pose name (snap, carry, home) or a specific camera to render"
---

# Scene Snapshot — VLA Tutorial

Render all 3 cameras in the current scene and save the frames to `outputs/`. Lets you visually verify the scene state without writing any new code.

## When to Use
- After editing any XML (world.xml, cage.xml, arm.xml, objects.xml)
- Before and after a physics change to compare visuals
- To verify camera orientations
- "Show me what the cameras see right now"
- Quick visual sanity check before starting a long simulation run

---

## Procedure

### Step 1 — Activate environment and set working directory
```bash
source ~/Desktop/phase4/phase4_env/bin/activate
cd ~/Desktop/VLA_Tutorial/VLATraining/sim
```

---

### Step 2 — Run the camera utils standalone test
`camera_utils.py` has a built-in `__main__` block that does exactly this:

```bash
python -m scene.camera_utils
```

This will:
1. Load `scene/world.xml`
2. Drive the arm to the snap pose (arm over the box)
3. Render `cam_overhead`, `cam_side`, `cam_wrist` at 640×480
4. Save to `outputs/`:
   - `outputs/cam_overhead.png`
   - `outputs/cam_side.png`
   - `outputs/cam_wrist.png`
5. Print shape assertions to confirm non-black frames

---

### Step 3 — Display the images
```bash
eog outputs/cam_overhead.png outputs/cam_side.png outputs/cam_wrist.png &
```
Or use any image viewer. The `&` keeps the terminal free.

If `eog` is not available:
```bash
xdg-open outputs/cam_overhead.png
```

---

### Step 4 — Save a timestamped snapshot (optional)
To keep a dated copy for comparison:

```bash
DATE=$(date +%Y-%m-%d)
cp outputs/cam_overhead.png outputs/snapshot_${DATE}_overhead.png
cp outputs/cam_side.png     outputs/snapshot_${DATE}_side.png
cp outputs/cam_wrist.png    outputs/snapshot_${DATE}_wrist.png
echo "Saved snapshot for $DATE"
```

---

### Step 5 — Report what you see
After rendering, briefly describe each camera view:

```
cam_overhead : looking straight down — arm, table, and box visible ✓ / ✗
cam_side     : looking toward workspace from right side — arm profile visible ✓ / ✗
cam_wrist    : egocentric, 20° downward tilt — end-effector and box visible ✓ / ✗
```

If any camera looks wrong (ceiling instead of floor, black frame, wrong angle), invoke `/agent-dumber` to diagnose.

---

## Camera reference

| Camera | Body | Position | Render direction |
|--------|------|----------|-----------------|
| `cam_overhead` | worldbody | `0.1 -0.05 2.0` | straight down (0,0,-1) |
| `cam_side` | cage_rail | `1.4 -0.3 1.35` | toward workspace (-0.95, 0, -0.30) |
| `cam_wrist` | em_pad | `-0.02 0 0.10` | forward + 20° down along arm |

## xyaxes quick reference (do not change unless re-diagnosing)
```xml
<!-- overhead -->  xyaxes="1 0 0  0 1 0"
<!-- side     -->  xyaxes="0 1 0  -0.306 0 0.952"
<!-- wrist    -->  xyaxes="0 1 0  -0.342 0 -0.940"
```

The render direction = `-(X_cam × Y_cam)`. To recompute:
```python
import numpy as np
X = np.array([...])  # first 3 values of xyaxes
Y = np.array([...])  # last 3 values of xyaxes
render_dir = -np.cross(X, Y)
print(render_dir)    # should point toward the scene
```
