---
description: "Verify all simulation sensors are publishing at correct rates and values. Use when: completing M4, checking sensor suite after any scene change, or debugging incorrect sensor readings."
name: "Verify Sensors"
argument-hint: "Path to scene XML (e.g. scene/world.xml)"
agent: "agent"
tools: ["read_file", "run_in_terminal"]
---

Verify the sensor suite for scene: **$SCENE_XML**

```bash
source /home/shahar/Desktop/phase4/phase4_env/bin/activate
cd /home/shahar/Desktop/phase4/VLATraining/sim
```

## Sensor reference table

| Sensor name | MuJoCo type | Expected range | Publish rate |
|---|---|---|---|
| `joint_pos_A1`–`A6` | `jointpos` | within joint limits | 100 Hz |
| `joint_vel_A1`–`A6` | `jointvel` | near 0 at rest | 100 Hz |
| `joint_torque_A1`–`A6` | `actuatorfce` | ≠ 0 when arm extended | 100 Hz |
| `ft_force` | `force` on EE site | ≥ 4.5 N Fz during EM hold | 100 Hz |
| `ft_torque` | `torque` on EE site | near 0 during free motion | 100 Hz |
| `proximity` | `rangefinder` on EE site | < 0.02 m when touching box | 50 Hz |
| `target_touch` | `touch` on target geom | > 0 when box on target | 50 Hz |

## Check 1 — Sensor index inventory

```python
import mujoco
model = mujoco.MjModel.from_xml_path("$SCENE_XML")
print(f"Total sensors: {model.nsensor}")
for i in range(model.nsensor):
    s = model.sensor(i)
    print(f"  [{i:2d}] {s.name:30s}  type={s.type}  adr={model.sensor_adr[i]}  dim={model.sensor_dim[i]}")
```

List all sensors found. Flag any expected sensor that is missing.

## Check 2 — Resting state values

Run 500 steps with arm at home pose (no commands applied). Print all sensordata values:

```python
import mujoco, numpy as np

model = mujoco.MjModel.from_xml_path("$SCENE_XML")
data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

for _ in range(500):
    mujoco.mj_step(model, data)

print("sensordata at rest:")
for i in range(model.nsensor):
    s = model.sensor(i)
    adr = model.sensor_adr[i]
    dim = model.sensor_dim[i]
    vals = data.sensordata[adr:adr+dim]
    print(f"  {s.name:30s} = {np.round(vals, 4)}")
```

## Check 3 — Proximity sensor approach test

Move the arm toward the box until the end-effector is < 2 cm from the box surface.
Read `proximity` sensor. Expected: value < 0.02.

```python
# After positioning arm close to box:
prox_id = model.sensor("proximity").id
adr = model.sensor_adr[prox_id]
distance = data.sensordata[adr]
print(f"Proximity distance: {distance:.4f} m  (expected < 0.02)")
assert distance < 0.02, f"FAIL: proximity {distance:.4f} >= 0.02 m"
print("PASS: proximity sensor")
```

## Check 4 — F/T sensor during EM hold

Activate the EM weld constraint, run 200 steps, read the vertical force component (Fz = index 2 of the force sensor):

```python
# Activate EM
em_id = model.equality("em_weld").id
model.eq_active[em_id] = True

for _ in range(200):
    mujoco.mj_step(model, data)

ft_id = model.sensor("ft_force").id
adr = model.sensor_adr[ft_id]
fz = data.sensordata[adr + 2]   # Z component
print(f"F/T Fz during EM hold: {fz:.3f} N  (expected >= 4.5 N)")
assert abs(fz) >= 4.5, f"FAIL: |Fz| = {abs(fz):.3f} N < 4.5 N threshold"
print("PASS: F/T sensor load check (box weight ≈ 0.5 kg × 9.81 = 4.9 N)")
```

## Check 5 — Target contact sensor

Place the box on the target zone geom. Run 200 steps. Read `target_touch`:

```python
touch_id = model.sensor("target_touch_sensor").id
adr = model.sensor_adr[touch_id]
val = data.sensordata[adr]
print(f"Target touch sensor: {val:.4f}  (expected > 0)")
assert val > 0, "FAIL: target touch sensor reads 0 with box on target"
print("PASS: target contact sensor")
```

## Summary

Report a table:

| Check | Result | Actual value | Expected |
|---|---|---|---|
| All expected sensors present | PASS/FAIL | count found | 19+ |
| Joint encoders at rest ≈ 0 | PASS/FAIL | | near 0 |
| Proximity < 0.02 m on approach | PASS/FAIL | | < 0.02 |
| F/T Fz ≥ 4.5 N during EM hold | PASS/FAIL | | ≥ 4.5 N |
| Target touch > 0 on contact | PASS/FAIL | | > 0 |
