"""
SESSION 1 — Opening the World: MuJoCo Real-Time Viewer
Phase 4 | April 13, 2026

What you will see:
  - Robotic environments opening in a live graphical window
  - Physics running in real time
  - 4 different environments, one after another

What you will learn:
  - What a joint / actuator / body is — visually
  - What a "passive viewer" is and why it matters for simulation
  - The foundation for Session 2 where you take control

Run: python3 session1_viewer_basics.py
Close each env window: Esc or click X
"""

import numpy as np
import mujoco
import mujoco.viewer
import time

print()
print("=" * 65)
print("  SESSION 1 — MuJoCo Real-Time Viewer")
print("=" * 65)
print("""
  4 windows will open, one after another.
  Each window = a different environment.
  Close each window (Esc / X) to move to the next.
""")


# ════════════════════════════════════════════════════════════
# ENV 1: Pendulum — the classic example
# ════════════════════════════════════════════════════════════

PENDULUM_XML = """
<mujoco model="pendulum">
  <option timestep="0.005" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.2 0.3 0.4 1"/>
    <body name="pivot" pos="0 0 1.2">
      <joint name="hinge" type="hinge" axis="1 0 0" limited="false" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0  0 0 -0.6"
            size="0.025" rgba="0.4 0.6 0.9 1"/>
      <body name="mass" pos="0 0 -0.6">
        <geom type="sphere" size="0.06" rgba="0.9 0.3 0.3 1" mass="1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="2" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

print("─" * 65)
print("  ENV 1/4 — Pendulum")
print("─" * 65)
print("""
  What to notice:
  * Red sphere   = mass — pulled down by gravity
  * Blue capsule = rigid rod
  * Pivot point  = where the rod is attached to the ceiling

  Key concepts:
  ┌──────────┬──────────────────────────────────────────┐
  │ joint    │ axis of motion — here: hinge = rotation  │
  │ body     │ physical object with mass and position   │
  │ geom     │ shape of the body (sphere, capsule, ...) │
  │ actuator │ the motor — what causes movement         │
  └──────────┴──────────────────────────────────────────┘

  No control in this simulation — gravity acts alone.
  Session 2: you will add keyboard control.
""")
input("  Press Enter to open the window...")

model_p = mujoco.MjModel.from_xml_string(PENDULUM_XML)
data_p  = mujoco.MjData(model_p)

# Initial kick — push the pendulum slightly
data_p.qpos[0] = np.pi * 0.9   # nearly upright — will fall dramatically
data_p.qvel[0] = 0.3

with mujoco.viewer.launch_passive(model_p, data_p) as viewer:
    viewer.cam.distance = 2.5
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 90
    start = time.time()
    print("  [window open] Watch the motion... Esc to continue")
    while viewer.is_running() and time.time() - start < 60:
        step_start = time.time()
        mujoco.mj_step(model_p, data_p)
        viewer.sync()
        # Real-time pacing — sleep for the remainder of the timestep
        elapsed = time.time() - step_start
        time.sleep(max(0, model_p.opt.timestep - elapsed))


# ════════════════════════════════════════════════════════════
# ENV 2: 2-joint robot arm (Reacher style)
# ════════════════════════════════════════════════════════════

REACHER_XML = """
<mujoco model="reacher">
  <option timestep="0.005" gravity="0 0 0"/>
  <visual>
    <rgba haze="0.1 0.1 0.2 1"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.5 0.5 0.5"/>
  </visual>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom type="plane" size="1 1 0.05" rgba="0.15 0.15 0.2 1"/>

    <!-- Base (fixed) -->
    <body name="base" pos="0 0 0.1">
      <geom type="cylinder" size="0.04 0.04" rgba="0.5 0.5 0.5 1"/>

      <!-- Link 1: Shoulder -->
      <body name="link1" pos="0 0 0.04">
        <joint name="j1" type="hinge" axis="0 0 1" damping="0.2"/>
        <geom type="capsule" fromto="0 0 0  0.2 0 0"
              size="0.025" rgba="0.3 0.6 0.9 1"/>
        <geom type="sphere" pos="0 0 0" size="0.03" rgba="0.9 0.5 0.1 1"/>

        <!-- Link 2: Elbow -->
        <body name="link2" pos="0.2 0 0">
          <joint name="j2" type="hinge" axis="0 0 1" damping="0.2"/>
          <geom type="capsule" fromto="0 0 0  0.15 0 0"
                size="0.020" rgba="0.3 0.6 0.9 1"/>
          <geom type="sphere" pos="0 0 0" size="0.025" rgba="0.9 0.5 0.1 1"/>

          <!-- Fingertip -->
          <body name="fingertip" pos="0.15 0 0">
            <geom type="sphere" size="0.020" rgba="1 0.9 0 1"/>
            <site name="ee" size="0.01"/>
          </body>
        </body>
      </body>
    </body>

    <!-- Target (red ball) -->
    <body name="target" pos="0.25 0.2 0.1">
      <geom type="sphere" size="0.025" rgba="0.9 0.1 0.1 0.8"
            contype="0" conaffinity="0"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="m1" joint="j1" gear="1" ctrlrange="-1 1"/>
    <motor name="m2" joint="j2" gear="1" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

print("\n" + "─" * 65)
print("  ENV 2/4 — 2DOF Robot Arm (Reacher)")
print("─" * 65)
print("""
  What to notice:
  * Yellow sphere = fingertip (end of the arm)
  * Red sphere    = target
  * 2 joints: j1 = shoulder, j2 = elbow

  The arm moves in a sinusoidal pattern.
  Think: which joint causes which motion?
  Session 2: you will control this with the keyboard.
""")
input("  Press Enter to open the window...")

model_r = mujoco.MjModel.from_xml_string(REACHER_XML)
data_r  = mujoco.MjData(model_r)

with mujoco.viewer.launch_passive(model_r, data_r) as viewer:
    viewer.cam.distance = 1.2
    viewer.cam.elevation = -70   # top-down view
    viewer.cam.azimuth   = 90
    start = time.time()
    t = 0
    print("  [window open] Watching motion... Esc to continue")
    while viewer.is_running() and time.time() - start < 60:
        step_start = time.time()
        t += model_r.opt.timestep
        # Sinusoidal control — watch the joints oscillate
        data_r.ctrl[0] = 0.4 * np.sin(2 * t)          # shoulder
        data_r.ctrl[1] = 0.3 * np.sin(3 * t + 1.0)    # elbow
        mujoco.mj_step(model_r, data_r)
        viewer.sync()
        elapsed = time.time() - step_start
        time.sleep(max(0, model_r.opt.timestep - elapsed))


# ════════════════════════════════════════════════════════════
# ENV 3: dm_control Cheetah — running
# ════════════════════════════════════════════════════════════

print("\n" + "─" * 65)
print("  ENV 3/4 — dm_control: Cheetah Run")
print("─" * 65)
print("""
  One of the most famous environments in VLA research.
  Cheetah = a 6-DOF robot that learns to run.

  What to notice:
  * Full body with gravity, friction, multi-dimensional motion
  * Each joint = one degree of freedom = one control dimension
  * Random policy = chaotic motion — this is the baseline

  In VLA: a policy receiving "run fast" produces commands for 6 joints.
""")
input("  Press Enter to open the window...")

try:
    from dm_control import suite as dm_suite
    env_ch = dm_suite.load('cheetah', 'run')
    ts = env_ch.reset()

    # Get the underlying MuJoCo model/data
    physics = env_ch.physics
    m_ch = physics.model._model  # underlying mujoco.MjModel
    d_ch = physics.data._data    # underlying mujoco.MjData

    with mujoco.viewer.launch_passive(m_ch, d_ch) as viewer:
        viewer.cam.distance = 4
        viewer.cam.elevation = -15
        viewer.cam.azimuth = 90
        start = time.time()
        print("  [window open] Random policy running... Esc to continue")
        while viewer.is_running() and time.time() - start < 60:
            step_start = time.time()
            action = env_ch.action_spec()
            random_action = np.random.uniform(
                action.minimum, action.maximum, action.shape
            )
            ts = env_ch.step(random_action)
            if ts.last():
                ts = env_ch.reset()
            viewer.sync()
            elapsed = time.time() - step_start
            time.sleep(max(0, 0.005 - elapsed))

except Exception as e:
    print(f"  ⚠️  dm_control Cheetah: {e}")
    print(f"  Skipping to ENV 4...")


# ════════════════════════════════════════════════════════════
# ENV 4: Gymnasium FetchReach — manipulation
# ════════════════════════════════════════════════════════════

print("\n" + "─" * 65)
print("  ENV 4/4 — Gymnasium: FetchReach")
print("─" * 65)
print("""
  FetchReach = a Fetch robot arm trying to reach a red ball.
  This is the environment closest to real VLA tasks:

  obs          = robot state (25 dim)
  desired_goal = where to go (3 dim: x, y, z)
  action       = [dx, dy, dz, dgrip] — delta position + gripper

  Watch the red ball change position between episodes.
  That ball is the "language instruction" encoded as a 3D position.
""")
input("  Press Enter to open the window...")

try:
    import gymnasium as gym
    import gymnasium_robotics  # noqa

    env_f = gym.make("FetchReach-v4", render_mode="human")
    obs, _ = env_f.reset(seed=0)

    print("  [window open] Random agent... Esc to finish")
    episode = 0
    for step in range(500):
        action = env_f.action_space.sample()
        obs, reward, terminated, truncated, info = env_f.step(action)
        if terminated or truncated:
            episode += 1
            obs, _ = env_f.reset(seed=episode)
            print(f"    Episode {episode} | "
                  f"success: {info.get('is_success', False)}")
    env_f.close()

except Exception as e:
    print(f"  ⚠️  FetchReach: {e}")
    print(f"  Try: pip install 'gymnasium[robotics]'")


# ════════════════════════════════════════════════════════════
# SUMMARY
# ════════════════════════════════════════════════════════════

print()
print("=" * 65)
print("  SESSION 1 COMPLETE ✅")
print("=" * 65)
print("""
  You saw:
  ──────
  [ok] Pendulum        — gravity, single joint, rotation axis
  [ok] 2DOF arm        — multiple joints, sinusoidal control
  [ok] Cheetah         — multi-DOF physics, random policy
  [ok] FetchReach      — goal-conditioned, realistic manipulation

  Core concept:
  ────────────────────
  joint → one motion dimension → ctrl[i] controls it
  body  → physical object with mass
  site  → reference point (end-effector, target)

  Session 2 — you take control:
  ──────────────────────────────
  Everything you just watched — you can control with the keyboard.
  A/D / Arrows = real-time joint control.
""")
