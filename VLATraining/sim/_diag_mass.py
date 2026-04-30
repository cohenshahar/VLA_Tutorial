import math, mujoco, numpy as np

# ── TEST C: kinematic force-lock — reset A2-A6 qpos+qvel every step ──────
# This bypasses ALL actuator/constraint mechanism for locking.
# If A1 moves here, the issue is HOW we lock other joints.
# If A1 still doesn't move, joint_a1 itself is broken.

model = mujoco.MjModel.from_xml_path("scene/world_test.xml")
data  = mujoco.MjData(model)

act_id = model.actuator("act_a1").id
jnt    = model.joint("joint_a1")
KP, KV = 800.0, 80.0
model.actuator_gainprm[act_id, 0] =  KP
model.actuator_biasprm[act_id, 1] = -KP
model.actuator_biasprm[act_id, 2] = -KV

LOCK = {"joint_a2": -45, "joint_a3": 45, "joint_a4": 0, "joint_a5": 0, "joint_a6": 0}
LOCK_INFO = {name: (model.joint(name).qposadr[0], model.joint(name).dofadr[0],
                    math.radians(deg))
             for name, deg in LOCK.items()}

# disable actuators for locked joints
for jname in LOCK:
    ai = model.actuator(f"act_{jname[-2:]}").id
    model.actuator_gainprm[ai, 0] = 0.0
    model.actuator_biasprm[ai, 1] = 0.0
    model.actuator_biasprm[ai, 2] = 0.0

for qaddr, daddr, q in LOCK_INFO.values():
    data.qpos[qaddr] = q
mujoco.mj_forward(model, data)

TARGET_RAD = math.radians(60.0)
print("=== TEST C: kinematic force-lock (qpos+qvel forced every step) ===")
print(f"{'step':>6}  {'t':>6}  {'theta_A1':>10}  {'theta_A2':>10}  {'force_A1':>10}")
for step in range(3000):
    # Enforce lock before physics step
    for qaddr, daddr, q in LOCK_INFO.values():
        data.qpos[qaddr] = q
        data.qvel[daddr] = 0.0
    ramp = min(data.time / 1.0, 1.0)
    data.ctrl[act_id] = ramp * TARGET_RAD
    mujoco.mj_step(model, data)
    if step % 300 == 0:
        t1 = math.degrees(data.qpos[jnt.qposadr[0]])
        t2 = math.degrees(data.qpos[model.joint("joint_a2").qposadr[0]])
        f  = data.actuator_force[act_id]
        print(f"{step:>6}  {data.time:>6.3f}  {t1:>+10.3f}  {t2:>+10.3f}  {f:>+10.1f}")

print()
print("=== Joint A1 info ===")
print(f"  jnt_type  = {model.jnt_type[jnt.id]}  (3=hinge, expected 3)")
print(f"  jnt_range = {model.jnt_range[jnt.id]}  rad")
print(f"  jnt_limited = {model.jnt_limited[jnt.id]}")
print(f"  dof_damping[0] = {model.dof_damping[0]}")
print(f"  dof_armature[0] = {model.dof_armature[0]}")
print(f"  dof_frictionloss[0] = {model.dof_frictionloss[0]}")
import sys; sys.exit(0)


model = mujoco.MjModel.from_xml_path("scene/world_test.xml")
data  = mujoco.MjData(model)

act_id = model.actuator("act_a1").id
jnt    = model.joint("joint_a1")
dof_id = jnt.dofadr[0]

KP, KV = 800.0, 80.0
model.actuator_gainprm[act_id, 0] =  KP
model.actuator_biasprm[act_id, 1] = -KP
model.actuator_biasprm[act_id, 2] = -KV

# ── TEST A: gravity OFF — if A1 moves, gravity+spring-locking is the problem ──
model.opt.gravity[:] = [0, 0, 0]

jnt_a2 = model.joint("joint_a2")
jnt_a3 = model.joint("joint_a3")
data.qpos[jnt_a2.qposadr[0]] = math.radians(-45.0)
data.qpos[jnt_a3.qposadr[0]] = math.radians( 45.0)
mujoco.mj_forward(model, data)
for i in range(model.nu):
    if i != act_id:
        model.actuator_gainprm[i, 0] =  500.0
        model.actuator_biasprm[i, 1] = -500.0
        model.actuator_biasprm[i, 2] = -50.0
        jnt_i = model.joint(model.actuator_trnid[i, 0])
        data.ctrl[i] = data.qpos[jnt_i.qposadr[0]]

TARGET_RAD = math.radians(60.0)
print("=== TEST A: gravity=0, spring-locking ===")
print(f"{'step':>6}  {'t':>6}  {'theta_A1':>10}  {'theta_A2':>10}  {'force_A1':>10}")
for step in range(2000):
    ramp = min(data.time / 1.0, 1.0)
    data.ctrl[act_id] = ramp * TARGET_RAD
    mujoco.mj_step(model, data)
    if step % 200 == 0:
        t1 = math.degrees(data.qpos[jnt.qposadr[0]])
        t2 = math.degrees(data.qpos[jnt_a2.qposadr[0]])
        f  = data.actuator_force[act_id]
        print(f"{step:>6}  {data.time:>6.3f}  {t1:>+10.3f}  {t2:>+10.3f}  {f:>+10.1f}")

# ── TEST B: gravity ON, TIGHT JOINT LIMITS instead of spring-locking ──
model2 = mujoco.MjModel.from_xml_path("scene/world_test.xml")
data2  = mujoco.MjData(model2)

act_id2 = model2.actuator("act_a1").id
jnt2    = model2.joint("joint_a1")

model2.actuator_gainprm[act_id2, 0] =  KP
model2.actuator_biasprm[act_id2, 1] = -KP
model2.actuator_biasprm[act_id2, 2] = -KV

# Pin A2-A6 with tight joint limits instead of spring actuators
LOCK_JOINTS = {"joint_a2": -45, "joint_a3": 45,
               "joint_a4": 0,   "joint_a5": 0, "joint_a6": 0}
EPS = 0.001  # rad = 0.057°
for jname, adeg in LOCK_JOINTS.items():
    ji = model2.joint(jname)
    q  = math.radians(adeg)
    model2.jnt_limited[ji.id]    = 1
    model2.jnt_range[ji.id, 0]   = q - EPS
    model2.jnt_range[ji.id, 1]   = q + EPS
    data2.qpos[ji.qposadr[0]]    = q
    # disable the spring actuator for this joint
    ai = model2.actuator(f"act_{jname[-2:]}").id
    model2.actuator_gainprm[ai, 0] = 0.0
    model2.actuator_biasprm[ai, 1] = 0.0
    model2.actuator_biasprm[ai, 2] = 0.0

mujoco.mj_forward(model2, data2)

print()
print("=== TEST B: gravity=ON, tight joint-limit locking ===")
print(f"{'step':>6}  {'t':>6}  {'theta_A1':>10}  {'theta_A2':>10}  {'force_A1':>10}")
for step in range(2000):
    ramp = min(data2.time / 1.0, 1.0)
    data2.ctrl[act_id2] = ramp * TARGET_RAD
    mujoco.mj_step(model2, data2)
    if step % 200 == 0:
        t1 = math.degrees(data2.qpos[jnt2.qposadr[0]])
        t2 = math.degrees(data2.qpos[model2.joint("joint_a2").qposadr[0]])
        f  = data2.actuator_force[act_id2]
        print(f"{step:>6}  {data2.time:>6.3f}  {t1:>+10.3f}  {t2:>+10.3f}  {f:>+10.1f}")

