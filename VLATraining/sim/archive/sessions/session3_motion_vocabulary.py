"""
SESSION 3 — Motion Vocabulary: Know What to Ask For
Phase 4 | April 13, 2026

Goal: build a complete motion vocabulary.
Each "motion sentence" gets:
  1. A name and plain-language description
  2. The code that generates it
  3. Real-time visualization

Keys:
  1-9 / 0  = switch between motion types
  Space    = reset
  Q        = quit

Run: python3 session3_motion_vocabulary.py
"""

import numpy as np
import mujoco
import mujoco.viewer
import threading
import time

# Keyboard state
keys_held = set()
keys_lock = threading.Lock()

try:
    from pynput import keyboard as kb

    def on_press(key):
        try:
            char = key.char.lower() if hasattr(key, 'char') and key.char else str(key)
            with keys_lock:
                keys_held.add(char)
        except Exception:
            pass

    def on_release(key):
        try:
            char = key.char.lower() if hasattr(key, 'char') and key.char else str(key)
            with keys_lock:
                keys_held.discard(char)
        except Exception:
            pass

    kb.Listener(on_press=on_press, on_release=on_release).start()
except ImportError:
    pass


def is_pressed(*chars):
    with keys_lock:
        return any(c in keys_held for c in chars)


# ──────────────────────────────────────────────────────────────────
# ROBOT MODEL
# ──────────────────────────────────────────────────────────────────

ARM_XML = """
<mujoco model="motion_vocab">
  <option timestep="0.005" gravity="0 0 -4.0"/>
  <visual>
    <rgba haze="0.04 0.08 0.18 1"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.55 0.55 0.55"/>
  </visual>
  <worldbody>
    <light pos="0.5 0.5 3" dir="-0.2 -0.2 -1" diffuse="0.7 0.7 0.7"/>
    <geom type="plane" size="2 2 0.05" rgba="0.1 0.1 0.15 1"/>

    <body name="base" pos="0 0 0.12">
      <geom type="cylinder" size="0.06 0.07" rgba="0.35 0.35 0.45 1"/>

      <body name="link1" pos="0 0 0.07">
        <joint name="j1" type="hinge" axis="0 0 1" damping="1.2" limited="false"/>
        <geom type="capsule" fromto="0 0 0  0.26 0 0"
              size="0.028" rgba="0.25 0.5 0.9 1"/>
        <geom type="sphere" pos="0 0 0" size="0.038" rgba="0.9 0.5 0.1 1"/>

        <body name="link2" pos="0.26 0 0">
          <joint name="j2" type="hinge" axis="0 1 0" damping="0.9"
                 range="-130 130"/>
          <geom type="capsule" fromto="0 0 0  0.20 0 0"
                size="0.022" rgba="0.25 0.5 0.9 1"/>
          <geom type="sphere" pos="0 0 0" size="0.030" rgba="0.9 0.5 0.1 1"/>

          <body name="link3" pos="0.20 0 0">
            <joint name="j3" type="hinge" axis="0 1 0" damping="0.6"
                   range="-90 90"/>
            <geom type="capsule" fromto="0 0 0  0.15 0 0"
                  size="0.018" rgba="0.3 0.6 0.85 1"/>
            <body name="fingertip" pos="0.15 0 0">
              <site name="ee" size="0.005"/>
              <geom type="sphere" size="0.022" rgba="1.0 0.9 0 1"/>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- 3 targets at different positions -->
    <body name="target_low"  pos=" 0.55  0.0  0.12">
      <geom type="sphere" size="0.028" rgba="0.9 0.15 0.15 0.7"
            contype="0" conaffinity="0"/>
      <site name="t_low" size="0.005"/>
    </body>
    <body name="target_mid"  pos=" 0.35  0.35 0.35">
      <geom type="sphere" size="0.028" rgba="0.15 0.85 0.15 0.7"
            contype="0" conaffinity="0"/>
      <site name="t_mid" size="0.005"/>
    </body>
    <body name="target_high" pos=" 0.0   0.0  0.70">
      <geom type="sphere" size="0.028" rgba="0.15 0.45 0.9 0.7"
            contype="0" conaffinity="0"/>
      <site name="t_high" size="0.005"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="m1" joint="j1" gear="5"  ctrlrange="-1 1"/>
    <motor name="m2" joint="j2" gear="5"  ctrlrange="-1 1"/>
    <motor name="m3" joint="j3" gear="3.5" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""


# ──────────────────────────────────────────────────────────────────
# MOTION VOCABULARY
# Each entry = name + description + function that computes ctrl
# ──────────────────────────────────────────────────────────────────

def motion_idle(t, qpos, qvel, **kw):
    """
    IDLE — standing still
    ctrl = 0 on all joints
    Motors apply no force — gravity acts freely
    """
    return np.zeros(3)


def motion_sweep_horizontal(t, qpos, qvel, **kw):
    """
    SWEEP HORIZONTAL — horizontal scan
    j1 (shoulder yaw) sweeps slowly end to end
    j2, j3 held fixed
    Like: "scan the environment left to right"
    """
    ctrl = np.zeros(3)
    target_j1 = 1.2 * np.sin(0.8 * t)
    error = target_j1 - qpos[0]
    ctrl[0] = np.clip(error * 2.5 - qvel[0] * 0.3, -1, 1)
    ctrl[1] = np.clip(-qpos[1] * 2 - qvel[1] * 0.2, -1, 1)  # hold j2
    ctrl[2] = np.clip(-qpos[2] * 2 - qvel[2] * 0.2, -1, 1)  # hold j3
    return ctrl


def motion_reach_forward(t, qpos, qvel, **kw):
    """
    REACH FORWARD — extend arm straight forward
    j2 (elbow) drops to zero — arm straight and pointing forward
    j1, j3 align to zero
    Like: "extend your arm straight out"
    """
    ctrl = np.zeros(3)
    # j1 → 0, j2 → 0 (straight), j3 → 0
    targets = [0.0, 0.0, 0.0]
    gains   = [3.0, 3.5, 2.5]
    for i in range(3):
        err = targets[i] - qpos[i]
        ctrl[i] = np.clip(err * gains[i] - qvel[i] * 0.4, -1, 1)
    return ctrl


def motion_reach_up(t, qpos, qvel, **kw):
    """
    REACH UP — raise arm upward
    j2 (elbow pitch) goes negative
    Like: "point upward"
    """
    ctrl = np.zeros(3)
    targets = [0.0, -1.1, 0.3]   # j2 = -1.1 rad = ~63° up
    gains   = [3.0, 4.0, 2.5]
    for i in range(3):
        err = targets[i] - qpos[i]
        ctrl[i] = np.clip(err * gains[i] - qvel[i] * 0.4, -1, 1)
    return ctrl


def motion_reach_target_low(t, qpos, qvel, model, data, **kw):
    """
    REACH TARGET LOW — reach the red ball (low)
    Computes Jacobian IK in real time
    Like: "touch the red point"
    """
    ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
    tgt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "t_low")
    ee_pos  = data.site_xpos[ee_id]
    tgt_pos = data.site_xpos[tgt_id]
    error   = tgt_pos - ee_pos

    jac = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jac, None, ee_id)
    jac_arm = jac[:, :model.nu]

    damping = 0.05
    ctrl = jac_arm.T @ np.linalg.solve(
        jac_arm @ jac_arm.T + damping * np.eye(3), error * 7.0
    )
    return np.clip(ctrl, -1, 1)


def motion_reach_target_mid(t, qpos, qvel, model, data, **kw):
    """
    REACH TARGET MID — reach the green ball (middle)
    Jacobian IK to a different point
    """
    ee_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee")
    tgt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "t_mid")
    ee_pos  = data.site_xpos[ee_id]
    tgt_pos = data.site_xpos[tgt_id]
    error   = tgt_pos - ee_pos

    jac = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jac, None, ee_id)
    jac_arm = jac[:, :model.nu]

    damping = 0.05
    ctrl = jac_arm.T @ np.linalg.solve(
        jac_arm @ jac_arm.T + damping * np.eye(3), error * 7.0
    )
    return np.clip(ctrl, -1, 1)


def motion_draw_circle(t, qpos, qvel, **kw):
    """
    DRAW CIRCLE — fingertip traces a circle
    j1 and j2 run sinusoids with 90 degree phase offset
    Like: "move in a circle"
    """
    ctrl = np.zeros(3)
    radius = 0.4
    speed  = 1.2
    target_j1 = radius * np.sin(speed * t)
    target_j2 = radius * np.sin(speed * t + np.pi / 2) * 0.5

    for i, (tgt, gain) in enumerate(zip(
            [target_j1, target_j2, 0.0], [3.0, 3.5, 2.5])):
        err = tgt - qpos[i]
        ctrl[i] = np.clip(err * gain - qvel[i] * 0.35, -1, 1)
    return ctrl


def motion_tremor(t, qpos, qvel, **kw):
    """
    TREMOR — shaking (simulates failure / disturbance)
    High noise + unpredictable motion
    Like: "the arm receives external disturbance"
    This is the failure mode your Verifier needs to detect!
    """
    ctrl = np.zeros(3)
    # Normal PD toward zero + high-frequency noise
    for i in range(3):
        err = -qpos[i]
        ctrl[i] = np.clip(err * 2.0 - qvel[i] * 0.3, -1, 1)
        ctrl[i] += np.random.normal(0, 0.5)   # noise!
    return np.clip(ctrl, -1, 1)


def motion_slow_fall(t, qpos, qvel, **kw):
    """
    SLOW FALL — controlled descent
    ctrl = 0 but damping slows the fall
    Like: "release the arm gradually"
    """
    # Just friction/damping → gravity pulls slowly
    ctrl = np.zeros(3)
    # Small counter-torque to slow the fall
    for i in range(3):
        ctrl[i] = np.clip(-qvel[i] * 0.4, -0.3, 0.3)
    return ctrl


def motion_wave(t, qpos, qvel, **kw):
    """
    WAVE — waving
    j1 oscillates fast, j2 + j3 fixed
    Like: "wave goodbye"
    """
    ctrl = np.zeros(3)
    target_j1 = 0.6 * np.sin(4.0 * t)   # faster oscillation
    target_j2 = -0.5                       # arm slightly up
    target_j3 = 0.3 * np.sin(6.0 * t)    # wrist wiggle

    targets = [target_j1, target_j2, target_j3]
    gains   = [4.0, 3.5, 3.0]
    for i in range(3):
        err = targets[i] - qpos[i]
        ctrl[i] = np.clip(err * gains[i] - qvel[i] * 0.3, -1, 1)
    return ctrl


# Motion registry
MOTIONS = {
    '1': {
        'name':    'IDLE — standing still',
        'desc':    'ctrl=0, gravity acts only',
        'fn':      motion_idle,
        'learn':   'ctrl[i] = 0 -> no force -> motor does not resist gravity',
    },
    '2': {
        'name':    'SWEEP HORIZONTAL',
        'desc':    'j1 runs sin, j2+j3 held',
        'fn':      motion_sweep_horizontal,
        'learn':   'target_angle -> error -> ctrl = Kp*error - Kd*velocity',
    },
    '3': {
        'name':    'REACH FORWARD — arm straight out',
        'desc':    'all joints targeting 0 (straight)',
        'fn':      motion_reach_forward,
        'learn':   'PD control to target angle: targets=[0,0,0]',
    },
    '4': {
        'name':    'REACH UP — raise arm',
        'desc':    'j2 goes to negative angle (~-1.1 rad)',
        'fn':      motion_reach_up,
        'learn':   'targets=[0, -1.1, 0.3] -> j2 in radians',
    },
    '5': {
        'name':    'REACH TARGET (red) — Jacobian IK',
        'desc':    'Jacobian IK to reach the red ball',
        'fn':      motion_reach_target_low,
        'learn':   'J_dagger * error -> ctrl (Cartesian space control)',
    },
    '6': {
        'name':    'REACH TARGET (green) — Jacobian IK',
        'desc':    'same mechanism, different target',
        'fn':      motion_reach_target_mid,
        'learn':   'changing target = only tgt_pos changes',
    },
    '7': {
        'name':    'DRAW CIRCLE',
        'desc':    'j1 + j2 sin with 90 deg phase offset',
        'fn':      motion_draw_circle,
        'learn':   'sin(t) + sin(t + pi/2) = circle in joint space',
    },
    '8': {
        'name':    'TREMOR — failure mode',
        'desc':    'high noise + PD -> instability',
        'fn':      motion_tremor,
        'learn':   'adding noise to ctrl = confidence drop in your thesis',
    },
    '9': {
        'name':    'SLOW FALL — controlled descent',
        'desc':    'damping only, no motor spring',
        'fn':      motion_slow_fall,
        'learn':   'ctrl = -Kd*vel (damping only, no spring)',
    },
    '0': {
        'name':    'WAVE',
        'desc':    'j1 fast + j2 fixed + j3 wiggle',
        'fn':      motion_wave,
        'learn':   'high freq (4.0) = faster motion',
    },
}


# ──────────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────────

print()
print("=" * 65)
print("  SESSION 3 — Motion Vocabulary")
print("=" * 65)

print("\n  Vocabulary:")
print(f"  {'Key':6s}  {'Name':40s}  {'Principle'}")
print(f"  {'─'*6}  {'─'*40}  {'─'*30}")
for key, m in MOTIONS.items():
    print(f"  [{key}]     {m['name']:40s}  {m['learn'][:40]}")

print("""
  Control Primitives overview:
  ─────────────────────────────────
  Every motion is built from one of 4 building blocks:

  1. ZERO:       ctrl[i] = 0
     -> no force, gravity acts freely

  2. PD:         ctrl[i] = Kp * (target - qpos[i]) - Kd * qvel[i]
     -> moves to target and stops, like a spring+damper

  3. SINUSOIDAL: ctrl[i] = A * sin(freq * t + phase)
     -> repeating wave motion; tune A=amplitude, freq=speed

  4. IK (Jacobian): ctrl = J_dagger * cartesian_error
     -> control end-effector position in 3D, not joints directly

  To request a new motion: just describe it.
  \"I want the fingertip to trace a figure-8\"
  -> sin + cos at 2 different frequencies
""")

input("  Press Enter to start...")

model = mujoco.MjModel.from_xml_string(ARM_XML)
data  = mujoco.MjData(model)

# Initial pose — arm slightly bent forward
data.qpos[:] = [0.0, -0.2, 0.2]

current_mode = ['1']
last_mode_print = [0.0]

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance  = 1.8
    viewer.cam.elevation = -22
    viewer.cam.azimuth   = 130

    t = 0.0
    print(f"\n  [window open] Current mode: {MOTIONS['1']['name']}")
    print("  Press 1-9/0 to switch | Enter=reset | Q=quit\n")

    while viewer.is_running():
        t += model.opt.timestep

        # ── Mode switching ────────────────────────────────
        for key in list(MOTIONS.keys()):
            if is_pressed(key) and key != current_mode[0]:
                current_mode[0] = key
                m = MOTIONS[key]
                print(f"\n  ▶ Mode {key}: {m['name']}")
                print(f"     {m['desc']}")
                print(f"     💡 {m['learn']}")
                last_mode_print[0] = time.time()

        if is_pressed('Key.enter'):
            mujoco.mj_resetData(model, data)
            data.qpos[:] = [0.0, -0.2, 0.2]
            t = 0.0
            print("  Reset")

        if is_pressed('q'):
            print("  Exiting Session 3")
            break

        # ── Compute control ───────────────────────────────
        fn = MOTIONS[current_mode[0]]['fn']
        ctrl = fn(
            t=t,
            qpos=data.qpos.copy(),
            qvel=data.qvel.copy(),
            model=model,
            data=data,
        )
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()


# ──────────────────────────────────────────────────────────────────
# SUMMARY
# ──────────────────────────────────────────────────────────────────

print()
print("=" * 65)
print("  SESSION 3 COMPLETE — Motion Vocabulary ready")
print("=" * 65)
print("""
  ─────────────────────────────────────────────────
  CHEAT SHEET — how to request motions in plain language
  ─────────────────────────────────────────────────

  "arm stand still"
  -> ctrl[:] = 0

  "arm sweep left/right slowly"
  -> target_j1 = A * sin(freq * t)
  -> small freq (0.5-1.0) = slow

  "reach point X, Y, Z"
  -> Jacobian IK with target = [X, Y, Z]
  -> ask: "what is the IK code for [0.4, 0.3, 0.5]?"

  "circular arm motion"
  -> j1 = sin(t), j2 = sin(t + pi/2)
  -> change radius: modify A

  "arm trembles / unstable"
  -> add: ctrl += np.random.normal(0, sigma, 3)
  -> sigma = tremor strength (0.1 = light, 0.8 = heavy)

  "raise arm to X degrees"
  -> targets[1] = np.radians(X) * -1   (j2, up = negative)
  -> change targets[1] = change height

  "wave"
  -> j1 = sin(4*t) high frequency
  -> j3 (wrist) = sin(6*t) for effect

  ─────────────────────────────────────────────────
  Parameters to remember:
  ─────────────────────────────────────────────────
  gear (in XML)    = overall gain. 5 = strong, 1 = soft
  damping (XML)    = how quickly motion is damped out
  Kp               = how hard to push toward target
  Kd               = how hard to brake velocity
  freq (in sin)    = cycles per second (rad/s)
  A (amplitude)    = sinusoid strength

  ─────────────────────────────────────────────────
  Next session:
  ─────────────────────────────────────────────────
  <- Replace SimulatedPolicy in the Verifier code
     with a real MotionVocabulary as a policy
  <- Define disturbance as tremor (mode 8) and check PLT
""")
