"""
Joint 1 step-response tuner — with LIVE MuJoCo viewer.

Run from VLATraining/sim/:
    python -m scene.tune_j1

A MuJoCo window opens. Watch the arm move in real time.
When the simulation finishes the window stays open — close it manually.

Tune the four numbers below, re-run, watch + check the plot.
Goal: smooth step 0° → +60°, minimal overshoot, fast settling.

Outputs:
    outputs/tune_j1_response.png   ← θ(t) + ω(t) plot
    outputs/tune_j1_response.mp4   ← recorded video of the motion
"""

import math, os, shutil, subprocess, time
import numpy as np
import mujoco
import mujoco.viewer
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
try:
    from PIL import Image
    def _imsave(pixels, path): Image.fromarray(pixels).save(path)
except ImportError:
    def _imsave(pixels, path): plt.imsave(path, pixels)

# ═══════════════════════════════════════════════════════════════════════════
#  TUNE THESE — re-run the script after each change
# ═══════════════════════════════════════════════════════════════════════════
KP        = 800.0   # position gain       [N·m / rad]
KV        = 80.0    # velocity gain (kd)  [N·m·s / rad]  ≈ 2√(KP·J)
DAMPING   = 10.0    # joint viscous damp  [N·m·s / rad]
ARMATURE  = 0.5     # motor rotor inertia [kg·m²]
# ═══════════════════════════════════════════════════════════════════════════

TARGET_DEG    = 60.0
RAMP_DURATION = 1.0    # setpoint ramps 0→target over this time (limits torque spike)
MAX_DURATION  = 10.0   # safety cap — stop even if gains are bad
SETTLE_POS    = 1.0    # °  — position tolerance to declare settled
SETTLE_VEL    = 2.0    # °/s — velocity tolerance to declare settled
VIDEO_FPS     = 60

# ── Paths ─────────────────────────────────────────────────────────────────
SIM_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD      = os.path.join(SIM_DIR, "scene", "world_test.xml")  # no table — clean physics test
OUT_DIR    = os.path.join(SIM_DIR, "outputs")
FRAMES_DIR = os.path.join(OUT_DIR, "tune_j1_frames")
PLOT_PATH  = os.path.join(OUT_DIR, "tune_j1_response.png")
VIDEO_PATH = os.path.join(OUT_DIR, "tune_j1_response.mp4")
os.makedirs(FRAMES_DIR, exist_ok=True)

# ── Load model + apply tuning params ──────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)

act_id = model.actuator("act_a1").id
jnt    = model.joint("joint_a1")
dof_id = jnt.dofadr[0]

model.actuator_gainprm[act_id, 0] =  KP
model.actuator_biasprm[act_id, 1] = -KP   # must match gainprm[0] for correct tracking
model.actuator_biasprm[act_id, 2] = -KV
model.dof_damping[dof_id]         =  DAMPING
model.dof_armature[dof_id]        =  ARMATURE

# Kinematic lock: disable actuators on locked joints — we force qpos+qvel each step.
# This is more reliable than spring locking when gravity is large (e.g. A2 at -45°).
LOCK_JOINTS = {"joint_a2": -45.0, "joint_a3": 45.0,
               "joint_a4":   0.0, "joint_a5":  0.0, "joint_a6": 0.0}
for i in range(model.nu):
    if i != act_id:
        model.actuator_gainprm[i, 0] = 0.0
        model.actuator_biasprm[i, 1] = 0.0
        model.actuator_biasprm[i, 2] = 0.0

model.vis.global_.offwidth  = 1280
model.vis.global_.offheight = 720

print(f"KP={KP}  KV={KV}  DAMPING={DAMPING}  ARMATURE={ARMATURE}")
print(f"Target: 0° → +{TARGET_DEG}°  (ramp {RAMP_DURATION}s, cap {MAX_DURATION}s)")
print("Watch the arm in the viewer window. Close it after simulation ends.")

target_rad    = math.radians(TARGET_DEG)
dt            = model.opt.timestep
steps_per_frm = max(1, int(round(1.0 / (VIDEO_FPS * dt))))

# Lock pose: A2=-45° (arm reaches forward), A3=+45° (elbow bent back up), rest=0°.
# This makes J1 base rotation clearly visible — the arm sweeps like a crane.
# A4-A6 stay at 0° (wrist neutral).
LOCK_INFO = {name: (model.joint(name).qposadr[0], model.joint(name).dofadr[0],
                    math.radians(deg))
             for name, deg in LOCK_JOINTS.items()}
for qaddr, daddr, q in LOCK_INFO.values():
    data.qpos[qaddr] = q
mujoco.mj_forward(model, data)   # propagate qpos before viewer opens

times, thetas, omegas = [], [], []
frame_idx = 0

# ── Off-screen camera for video ───────────────────────────────────────────
cam = mujoco.MjvCamera()
cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0.0, 0.0, 1.0]
cam.distance  = 2.5
cam.azimuth   = 180
cam.elevation = -15

# ── Run simulation with live viewer ───────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0.0, 0.0, 1.0]
    viewer.cam.distance  = 2.5
    viewer.cam.azimuth   = 180
    viewer.cam.elevation = -15

    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        wall_start = time.time()
        step    = 0
        settled = False
        while viewer.is_running() and not settled and data.time < MAX_DURATION:
            # Kinematic lock: hold A2-A6 at fixed angles every step
            for qaddr, daddr, q in LOCK_INFO.values():
                data.qpos[qaddr] = q
                data.qvel[daddr] = 0.0

            # Soft start: ramp setpoint 0 → target over RAMP_DURATION
            ramp = min(data.time / RAMP_DURATION, 1.0)
            data.ctrl[act_id] = ramp * target_rad

            mujoco.mj_step(model, data)

            t     = data.time
            theta = math.degrees(data.qpos[jnt.qposadr[0]])
            omega = math.degrees(data.qvel[dof_id])
            times.append(t)
            thetas.append(theta)
            omegas.append(omega)

            # Settled: close enough to target AND nearly still
            if ramp >= 1.0:  # only check after ramp is complete
                settled = (abs(theta - TARGET_DEG) < SETTLE_POS
                           and abs(omega) < SETTLE_VEL)

            # Throttle to real-time so viewer shows actual motion
            wall_target = wall_start + t
            sleep_time  = wall_target - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

            viewer.sync()

            # Capture frame for video
            if step % steps_per_frm == 0:
                renderer.update_scene(data, camera=cam)
                pixels = renderer.render()
                _imsave(pixels, os.path.join(FRAMES_DIR, f"frame_{frame_idx:05d}.png"))
                frame_idx += 1

            step += 1

            # Console update every 0.5 s
            if step % 500 == 0:
                print(f"  t={t:.2f}s  θ={theta:+.2f}°  ω={omega:+.1f}°/s  "
                      f"ncon={data.ncon}")

        if settled:
            print(f"\nSettled at t={data.time:.2f}s  θ={theta:.2f}°  (target {TARGET_DEG}°)")
        else:
            print(f"\nMax duration reached ({MAX_DURATION}s).  θ={theta:.2f}°")
    # Keep viewer open after sim finishes so user can inspect
    while viewer.is_running():
        viewer.sync()

# ── Metrics ───────────────────────────────────────────────────────────────
times  = np.array(times)
thetas = np.array(thetas)
omegas = np.array(omegas)

overshoot  = max(0.0, thetas.max() - TARGET_DEG)
band       = 0.02 * TARGET_DEG
settle_time = None
for i in range(len(thetas) - 1, -1, -1):
    if abs(thetas[i] - TARGET_DEG) > band:
        settle_time = times[i + 1] if i + 1 < len(thetas) else None
        break
if settle_time is None:
    settle_time = 0.0

print(f"\nResults:")
print(f"  Overshoot     : {overshoot:+.2f}°  ({100*overshoot/TARGET_DEG:.1f}%)")
print(f"  Settling time : {settle_time:.3f}s  (±2% = ±{band:.2f}°)")
print(f"  Final value   : {thetas[-1]:.2f}°  (target {TARGET_DEG}°)")
print(f"  Steady error  : {thetas[-1]-TARGET_DEG:.3f}°")

# ── Plot ──────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
fig.suptitle(
    f"J1 Step Response  0° → {TARGET_DEG}°\n"
    f"KP={KP}  KV={KV}  damping={DAMPING}  armature={ARMATURE}  "
    f"│  OS={overshoot:.1f}°  ts={settle_time:.2f}s",
    fontsize=12, fontweight="bold"
)
ax1.axhline(TARGET_DEG,        color="green", ls="--", lw=1.5, label=f"Target {TARGET_DEG}°")
ax1.axhline(TARGET_DEG * 1.02, color="grey",  ls=":",  lw=1,   label="±2% band")
ax1.axhline(TARGET_DEG * 0.98, color="grey",  ls=":",  lw=1)
ax1.plot(times, thetas, color="royalblue", lw=2, label="θ [deg]")
if settle_time > 0:
    ax1.axvline(settle_time, color="orange", ls="-.", lw=1.5,
                label=f"Settle {settle_time:.2f}s")
if overshoot > 0.5:
    peak_t = times[np.argmax(thetas)]
    ax1.annotate(f"OS={overshoot:.1f}°",
                 xy=(peak_t, thetas.max()),
                 xytext=(peak_t + 0.1, thetas.max() + 2),
                 arrowprops=dict(arrowstyle="->", color="red"),
                 color="red", fontsize=10)
ax1.set_ylabel("θ  [degrees]"); ax1.legend(loc="lower right"); ax1.grid(alpha=0.3)

ax2.plot(times, omegas, color="tomato", lw=1.5, label="ω [deg/s]")
ax2.axhline(0, color="black", lw=0.8)
ax2.set_ylabel("ω  [degrees/s]"); ax2.set_xlabel("Time  [s]")
ax2.legend(); ax2.grid(alpha=0.3)

plt.tight_layout()
fig.savefig(PLOT_PATH, dpi=120)
plt.close(fig)
print(f"Plot  → {PLOT_PATH}")

# ── Encode video ──────────────────────────────────────────────────────────
ffmpeg = shutil.which("ffmpeg")
if ffmpeg:
    cmd = [ffmpeg, "-y", "-framerate", str(VIDEO_FPS),
           "-i", os.path.join(FRAMES_DIR, "frame_%05d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", VIDEO_PATH]
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(f"Video → {VIDEO_PATH}" if r.returncode == 0 else f"ffmpeg error: {r.stderr[-200:]}")
shutil.rmtree(FRAMES_DIR)
print("\nDone. Adjust KP/KV/DAMPING/ARMATURE at top and re-run.")
