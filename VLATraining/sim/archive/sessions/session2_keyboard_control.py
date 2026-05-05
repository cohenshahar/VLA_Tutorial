"""
SESSION 2 — Keyboard Control: You Are the Policy
Phase 4 | April 13, 2026

You control the robot in real time with the keyboard.

PART A — 2DOF arm (easy, get a feel for it)
  ← / →     = shoulder joint (left/right)
  ↑ / ↓     = elbow joint (up/down)
  Enter     = reset
  Q         = quit to Part B

PART B — 3DOF arm (three degrees of freedom)
  ← / →     = joint 1 (shoulder yaw)
  ↑ / ↓     = joint 2 (elbow pitch)
  PgUp/PgDn = joint 3 (wrist)
  Enter     = reset

Run: python3 session2_keyboard_control.py
"""

import numpy as np
import mujoco
import mujoco.viewer
import threading
import time

# Keyboard state — tracks which keys are currently held
keys_held = set()
keys_lock = threading.Lock()

try:
    from pynput import keyboard as kb

    def on_press(key):
        try:
            with keys_lock:
                keys_held.add(key.char.lower() if hasattr(key, 'char') and key.char else str(key))
        except Exception:
            pass

    def on_release(key):
        try:
            with keys_lock:
                keys_held.discard(key.char.lower() if hasattr(key, 'char') and key.char else str(key))
        except Exception:
            pass

    keyboard_listener = kb.Listener(on_press=on_press, on_release=on_release)
    keyboard_listener.start()
    KEYBOARD_AVAILABLE = True

except ImportError:
    print("  pynput not installed. Running without keyboard control.")
    print("  Install: pip install pynput")
    KEYBOARD_AVAILABLE = False


def is_pressed(*chars):
    with keys_lock:
        return any(c in keys_held for c in chars)


def print_controls(part, controls):
    print(f"\n  {'─'*50}")
    print(f"  PART {part} — Controls:")
    for key, action in controls.items():
        print(f"    {key:12s} → {action}")
    print(f"  {'─'*50}\n")


# ════════════════════════════════════════════════════════════
# PART A — 2DOF arm: get a feel for joint control
# ════════════════════════════════════════════════════════════

REACHER_2DOF = """
<mujoco model="reacher_keyboard">
  <option timestep="0.005" gravity="0 0 0"/>
  <visual>
    <rgba haze="0.05 0.1 0.2 1"/>
    <headlight ambient="0.5 0.5 0.5" diffuse="0.5 0.5 0.5"/>
    <map znear="0.01"/>
  </visual>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom type="plane" size="1 1 0.05" rgba="0.12 0.12 0.18 1"/>

    <body name="base" pos="0 0 0.05">
      <geom type="cylinder" size="0.05 0.04" rgba="0.4 0.4 0.5 1"/>

      <body name="link1" pos="0 0 0.04">
        <joint name="j1" type="hinge" axis="0 0 1" damping="0.5" limited="false"/>
        <geom type="capsule" fromto="0 0 0  0.22 0 0"
              size="0.028" rgba="0.3 0.55 0.9 1"/>
        <geom type="sphere" pos="0 0 0" size="0.035" rgba="0.85 0.45 0.1 1"/>

        <body name="link2" pos="0.22 0 0">
          <joint name="j2" type="hinge" axis="0 0 1" damping="0.5" limited="false"/>
          <geom type="capsule" fromto="0 0 0  0.18 0 0"
                size="0.022" rgba="0.3 0.55 0.9 1"/>
          <geom type="sphere" pos="0 0 0" size="0.028" rgba="0.85 0.45 0.1 1"/>

          <body name="fingertip" pos="0.18 0 0">
            <geom type="sphere" size="0.022" rgba="1.0 0.85 0 1"/>
            <site name="ee" size="0.005"/>
          </body>
        </body>
      </body>
    </body>

    <!-- Target ball — same z-plane as the arm (z=0.09) -->
    <body name="target" pos="0.3 0.1 0.09">
      <geom type="sphere" size="0.028" rgba="0.9 0.15 0.15 0.75"
            contype="0" conaffinity="0"/>
      <site name="target_site" size="0.005"/>
    </body>
  </worldbody>

  <actuator>
    <!-- gear = gain. Higher = faster response -->
    <motor name="m1" joint="j1" gear="3" ctrlrange="-1 1"/>
    <motor name="m2" joint="j2" gear="3" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

print("\n" + "=" * 65)
print("  SESSION 2 — Keyboard Control")
print("=" * 65)
print("""
  You are now the policy.
  Your keyboard = ctrl[] commands to the joints.
  MuJoCo physics handles the rest.
""")

print_controls("A — 2DOF arm", {
    "← / →":          "shoulder (j1) — left/right",
    "↑ / ↓":          "elbow (j2) — up/down",
    "Enter":           "reset position",
    "Q":               "quit to Part B",
})

input("  Press Enter to start...")

model_a = mujoco.MjModel.from_xml_string(REACHER_2DOF)
data_a  = mujoco.MjData(model_a)

# IDs for distance calculation
ee_site_id     = mujoco.mj_name2id(model_a, mujoco.mjtObj.mjOBJ_SITE, "ee")
target_site_id = mujoco.mj_name2id(model_a, mujoco.mjtObj.mjOBJ_SITE, "target_site")

TORQUE = 0.25   # torque applied per keypress

hold_qpos_a = np.zeros(2)   # position to hold when no key pressed
KP_A, KD_A = 2.5, 0.4      # PD gains for hold

step_count = 0
min_dist   = float('inf')

with mujoco.viewer.launch_passive(model_a, data_a) as viewer:
    viewer.cam.distance  = 1.1
    viewer.cam.elevation = -75    # top-down — best view for 2D arm
    viewer.cam.azimuth   = 90

    print("  [window open] — Take control of the arm!")

    while viewer.is_running():
        # Key reading
        ctrl = np.zeros(model_a.nu)

        j1_key = False
        j2_key = False
        if is_pressed('Key.left'):
            ctrl[0] = -TORQUE    # j1 left
            j1_key = True
        if is_pressed('Key.right'):
            ctrl[0] = +TORQUE    # j1 right
            j1_key = True
        if is_pressed('Key.up'):
            ctrl[1] = +TORQUE    # j2 up
            j2_key = True
        if is_pressed('Key.down'):
            ctrl[1] = -TORQUE    # j2 down
            j2_key = True

        # PD hold — freeze at last position when key released
        if j1_key:
            hold_qpos_a[0] = data_a.qpos[0]
        else:
            ctrl[0] = np.clip(KP_A*(hold_qpos_a[0] - data_a.qpos[0]) - KD_A*data_a.qvel[0], -1, 1)
        if j2_key:
            hold_qpos_a[1] = data_a.qpos[1]
        else:
            ctrl[1] = np.clip(KP_A*(hold_qpos_a[1] - data_a.qpos[1]) - KD_A*data_a.qvel[1], -1, 1)

        if is_pressed('Key.enter'):   # Enter = reset
            mujoco.mj_resetData(model_a, data_a)
            hold_qpos_a[:] = 0.0
            print("  Reset")

        if is_pressed('q'):      # Q = quit
            print("  Moving to Part B")
            break

        data_a.ctrl[:] = ctrl

        # Physics step
        mujoco.mj_step(model_a, data_a)
        viewer.sync()
        step_count += 1

        # Distance to target
        ee_pos     = data_a.site_xpos[ee_site_id]
        target_pos = data_a.site_xpos[target_site_id]
        dist       = np.linalg.norm(ee_pos - target_pos)
        if dist < min_dist:
            min_dist = dist
            if dist < 0.05:
                print(f"  Target reached! dist={dist:.3f}m")

# ════════════════════════════════════════════════════════════
# PART B — 3DOF arm: 3 joints
# ════════════════════════════════════════════════════════════

ARM_3DOF = """
<mujoco model="arm3dof_keyboard">
  <option timestep="0.005" gravity="0 0 -4.0"/>
  <visual>
    <rgba haze="0.05 0.1 0.2 1"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.7 0.7 0.7"/>
    <geom type="plane" size="1.5 1.5 0.05" rgba="0.12 0.12 0.18 1"/>

    <body name="base" pos="0 0 0.12">
      <geom type="cylinder" size="0.055 0.06" rgba="0.35 0.35 0.45 1"/>

      <!-- Joint 1: Shoulder yaw (A/D) -->
      <body name="link1" pos="0 0 0.06">
        <joint name="j1" type="hinge" axis="0 0 1" damping="0.8" limited="false"/>
        <geom type="capsule" fromto="0 0 0  0.28 0 0"
              size="0.028" rgba="0.3 0.55 0.9 1"/>
        <geom type="sphere" pos="0 0 0"    size="0.038" rgba="0.85 0.45 0.1 1"/>

        <!-- Joint 2: Elbow pitch (W/S) -->
        <body name="link2" pos="0.28 0 0">
          <joint name="j2" type="hinge" axis="0 1 0" damping="0.6" range="-120 120"/>
          <geom type="capsule" fromto="0 0 0  0.22 0 0"
                size="0.022" rgba="0.3 0.55 0.9 1"/>
          <geom type="sphere" pos="0 0 0"    size="0.030" rgba="0.85 0.45 0.1 1"/>

          <!-- Joint 3: Wrist (E/X) -->
          <body name="link3" pos="0.22 0 0">
            <joint name="j3" type="hinge" axis="0 1 0" damping="0.4" range="-90 90"/>
            <geom type="capsule" fromto="0 0 0  0.16 0 0"
                  size="0.018" rgba="0.4 0.65 0.85 1"/>
            <body name="fingertip" pos="0.16 0 0">
              <geom type="sphere" size="0.022" rgba="1.0 0.85 0 1"/>
              <site name="ee" size="0.005"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Multiple targets at different positions -->
    <body name="target1" pos="0.45 0.0 0.12">
      <geom type="sphere" size="0.025" rgba="0.9 0.15 0.15 0.7"
            contype="0" conaffinity="0"/>
      <site name="t1" size="0.005"/>
    </body>
    <body name="target2" pos="0.3 0.35 0.12">
      <geom type="sphere" size="0.025" rgba="0.15 0.9 0.15 0.7"
            contype="0" conaffinity="0"/>
      <site name="t2" size="0.005"/>
    </body>
    <body name="target3" pos="0.0 0.45 0.3">
      <geom type="sphere" size="0.025" rgba="0.15 0.15 0.9 0.7"
            contype="0" conaffinity="0"/>
      <site name="t3" size="0.005"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="m1" joint="j1" gear="4" ctrlrange="-1 1"/>
    <motor name="m2" joint="j2" gear="4" ctrlrange="-1 1"/>
    <motor name="m3" joint="j3" gear="3" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

print_controls("B — 3DOF arm", {
    "← / →":          "j1 shoulder yaw (base rotation)",
    "↑ / ↓":          "j2 elbow pitch (raise/lower elbow)",
    "PgUp / PgDn":    "j3 wrist (wrist rotation)",
    "Enter":           "reset",
    "1/2/3":           "print joint angles + fingertip position",
    "Q":               "quit to Part C",
})

print("""
  Tasks to try:
  (1) Touch the red ball (front)
  (2) Touch the green ball (right side)
  (3) Touch the blue ball (high up)
  (4) Do a full sweep — j1 from end to end
""")
input("  Press Enter to start...")

model_b = mujoco.MjModel.from_xml_string(ARM_3DOF)
data_b  = mujoco.MjData(model_b)
ee_id_b = mujoco.mj_name2id(model_b, mujoco.mjtObj.mjOBJ_SITE, "ee")
t1_id   = mujoco.mj_name2id(model_b, mujoco.mjtObj.mjOBJ_SITE, "t1")
t2_id   = mujoco.mj_name2id(model_b, mujoco.mjtObj.mjOBJ_SITE, "t2")
t3_id   = mujoco.mj_name2id(model_b, mujoco.mjtObj.mjOBJ_SITE, "t3")
TARGET_IDS    = [t1_id, t2_id, t3_id]
TARGET_COLORS = ["RED", "GREEN", "BLUE"]

# Initial pose — arm stretched forward
data_b.qpos[:] = [0.0, -0.3, 0.3]

hold_qpos_b = data_b.qpos.copy()   # PD hold target
KP_B, KD_B = 2.5, 0.4
touched     = [False, False, False]

TORQUE_B = 0.3
last_info_print = 0

with mujoco.viewer.launch_passive(model_b, data_b) as viewer:
    viewer.cam.distance  = 1.8
    viewer.cam.elevation = -25
    viewer.cam.azimuth   = 135

    print("  [window open] — 3 joints, 3 colored targets!")

    while viewer.is_running():
        ctrl    = np.zeros(model_b.nu)
        key_j   = [False, False, False]

        # Joint 1 — shoulder yaw
        if is_pressed('Key.left'):  ctrl[0] = -TORQUE_B; key_j[0] = True
        if is_pressed('Key.right'): ctrl[0] = +TORQUE_B; key_j[0] = True

        # Joint 2 — elbow pitch
        if is_pressed('Key.up'):    ctrl[1] = +TORQUE_B; key_j[1] = True
        if is_pressed('Key.down'):  ctrl[1] = -TORQUE_B; key_j[1] = True

        # Joint 3 — wrist
        if is_pressed('Key.page_up'):   ctrl[2] = +TORQUE_B * 0.7; key_j[2] = True
        if is_pressed('Key.page_down'): ctrl[2] = -TORQUE_B * 0.7; key_j[2] = True

        # PD hold — freeze at last position when key released
        for i in range(3):
            if key_j[i]:
                hold_qpos_b[i] = data_b.qpos[i]
            else:
                ctrl[i] = np.clip(
                    KP_B*(hold_qpos_b[i] - data_b.qpos[i]) - KD_B*data_b.qvel[i],
                    -1, 1
                )

        if is_pressed('Key.enter'):
            mujoco.mj_resetData(model_b, data_b)
            data_b.qpos[:] = [0.0, -0.3, 0.3]
            hold_qpos_b[:] = data_b.qpos
            touched[:] = [False, False, False]
            print("  Reset")

        # Info print (every ~2 seconds)
        now = time.time()
        if is_pressed('1') or is_pressed('2') or is_pressed('3'):
            if now - last_info_print > 1.0:
                last_info_print = now
                ee = data_b.site_xpos[ee_id_b]
                q  = data_b.qpos
                print(f"  Joint angles: j1={np.degrees(q[0]):6.1f}°  "
                      f"j2={np.degrees(q[1]):6.1f}°  "
                      f"j3={np.degrees(q[2]):6.1f}°")
                print(f"  Fingertip pos: x={ee[0]:.3f}  y={ee[1]:.3f}  z={ee[2]:.3f}")

        if is_pressed('q'):
            print("  Moving to Part C")
            break

        data_b.ctrl[:] = ctrl
        mujoco.mj_step(model_b, data_b)
        viewer.sync()

        # Ball-touch detection
        ee = data_b.site_xpos[ee_id_b]
        for i, (t_id, color) in enumerate(zip(TARGET_IDS, TARGET_COLORS)):
            if not touched[i]:
                dist = np.linalg.norm(ee - data_b.site_xpos[t_id])
                if dist < 0.05:
                    touched[i] = True
                    score = sum(touched)
                    print(f"  *** {color} touched! ({score}/3) ***")
                    if score == 3:
                        print("  *** ALL 3 TARGETS REACHED! ***")


# ════════════════════════════════════════════════════════════
# PART C — Motion Modes: advanced concept
# ════════════════════════════════════════════════════════════

print("\n" + "─" * 65)
print("  PART C — Motion Modes: three movement strategies")
print("─" * 65)
print("""
  Now watch 3 different movement strategies on the same robot.
  This is what the "policy" decides — not you.

  Mode 1: SINUSOIDAL — smooth wave motion
  Mode 2: STEP       — snaps to position and holds
  Mode 3: RANDOM     — like Session 1

  Press 1/2/3 to switch modes in real time.
  Notice how "policy behavior" shapes the motion.
""")
input("  Press Enter to start...")

REACH_XML = """
<mujoco model="motion_modes">
  <option timestep="0.005" gravity="0 0 -3.0"/>
  <visual>
    <rgba haze="0.05 0.1 0.2 1"/>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom type="plane" size="1.5 1.5 0.05" rgba="0.12 0.12 0.18 1"/>
    <body name="base" pos="0 0 0.1">
      <geom type="cylinder" size="0.05 0.05" rgba="0.35 0.35 0.45 1"/>
      <body name="link1" pos="0 0 0.05">
        <joint name="j1" type="hinge" axis="0 0 1" damping="1.0" limited="false"/>
        <geom type="capsule" fromto="0 0 0  0.25 0 0"
              size="0.026" rgba="0.3 0.55 0.9 1"/>
        <geom type="sphere" pos="0 0 0" size="0.035" rgba="0.85 0.45 0.1 1"/>
        <body name="link2" pos="0.25 0 0">
          <joint name="j2" type="hinge" axis="0 0 1" damping="0.8" limited="false"/>
          <geom type="capsule" fromto="0 0 0  0.20 0 0"
                size="0.020" rgba="0.3 0.55 0.9 1"/>
          <geom type="sphere" pos="0 0 0" size="0.028" rgba="0.85 0.45 0.1 1"/>
          <body name="fingertip" pos="0.20 0 0">
            <geom type="sphere" size="0.020" rgba="1.0 0.85 0 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m1" joint="j1" gear="3" ctrlrange="-1 1"/>
    <motor name="m2" joint="j2" gear="3" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

model_c = mujoco.MjModel.from_xml_string(REACH_XML)
data_c  = mujoco.MjData(model_c)

MODES = {
    '1': 'SINUSOIDAL',
    '2': 'STEP',
    '3': 'RANDOM',
}
current_mode = ['1']

print_controls("C — Motion Modes", {
    "1": "SINUSOIDAL (smooth waves)",
    "2": "STEP (snaps to target)",
    "3": "RANDOM (chaotic)",
    "Enter": "reset",
    "Q": "end session",
})

with mujoco.viewer.launch_passive(model_c, data_c) as viewer:
    viewer.cam.distance  = 1.2
    viewer.cam.elevation = -70
    viewer.cam.azimuth   = 90
    t = 0.0
    step_target = [0.5, -0.5]
    step_timer  = [0]

    print(f"  [window open] Mode: {MODES[current_mode[0]]}")
    last_mode_print = [time.time()]

    while viewer.is_running():
        t += model_c.opt.timestep

        # Switch modes
        for key in ['1', '2', '3']:
            if is_pressed(key) and key != current_mode[0]:
                current_mode[0] = key
                step_timer[0] = 0   # reset timer so new mode starts clean
                print(f"  Mode → {MODES[key]}")

        # Periodic reminder of current mode
        now_c = time.time()
        if now_c - last_mode_print[0] > 5.0:
            last_mode_print[0] = now_c
            print(f"  [mode: {MODES[current_mode[0]]}]  press 1/2/3 to switch")

        if is_pressed('Key.enter'):
            mujoco.mj_resetData(model_c, data_c)
            t = 0.0

        if is_pressed('q'):
            print("  Finished Part C")
            break

        # Compute control based on mode
        mode = current_mode[0]
        if mode == '1':   # Sinusoidal — smooth, rhythmic
            data_c.ctrl[0] = 0.5 * np.sin(1.5 * t)
            data_c.ctrl[1] = 0.4 * np.sin(2.5 * t + 0.8)

        elif mode == '2':  # Step — move to position, hold
            step_timer[0] += 1
            if step_timer[0] > 200:
                step_target[0] *= -1
                step_target[1] *= -1
                step_timer[0] = 0
                print(f"  Step target: j1={step_target[0]:.1f}  j2={step_target[1]:.1f}")
            # PD control toward step target
            err0 = step_target[0] - data_c.qpos[0]
            err1 = step_target[1] - data_c.qpos[1]
            data_c.ctrl[0] = np.clip(err0 * 2.0 - data_c.qvel[0] * 0.3, -1, 1)
            data_c.ctrl[1] = np.clip(err1 * 2.0 - data_c.qvel[1] * 0.3, -1, 1)

        elif mode == '3':  # Random
            if step_timer[0] % 15 == 0:
                data_c.ctrl[:] = np.random.uniform(-0.6, 0.6, model_c.nu)
            step_timer[0] += 1

        mujoco.mj_step(model_c, data_c)
        viewer.sync()

# ════════════════════════════════════════════════════════════
# SUMMARY
# ════════════════════════════════════════════════════════════

print()
print("=" * 65)
print("  SESSION 2 COMPLETE — You controlled the robot!")
print("=" * 65)
print("""
  What you did:
  ─────────────
  [ok] Controlled joint torques in real time with A/D, arrows, E/X
  [ok] Saw how gear (gain) affects response speed
  [ok] Learned: each key = ctrl[i] command to a joint
  [ok] Saw 3 motion styles: sinusoidal / step / random

  Core concept:
  ──────────────────────
  ctrl[i] = force/torque magnitude on joint i
  positive / negative = direction of motion
  higher = faster motion

  Session 3:
  ────────────
  Build a "motion vocabulary" — how to describe in plain language
  each type of motion, and what code generates it.
  This is "prompt engineering" for robots.
""")
