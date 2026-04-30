"""
Phase 3 — Task 3.9: Smooth sinusoidal A1 trajectory → MP4 video.

Run from VLATraining/sim/:
    python -m scene.record_trajectory

What it does:
  - Drives joint A1 through ±60° sinusoidal oscillation over 4 seconds
  - Renders every physics step at 30 fps (saves 1 frame per ~33 ms of sim)
  - Writes frames to outputs/frames_A1/ (temp, auto-cleaned after encode)
  - Encodes to outputs/joint_A1_oscillation.mp4 using ffmpeg

Done when:
  - outputs/joint_A1_oscillation.mp4 exists and plays correctly
  - Arm oscillates smoothly ±60° in the video
"""
import math
import os
import shutil
import subprocess
import numpy as np
import mujoco

try:
    from PIL import Image
    def _save(pixels, path):
        Image.fromarray(pixels).save(path)
except ImportError:
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    def _save(pixels, path):
        plt.imsave(path, pixels)

# ── Paths ────────────────────────────────────────────────────────────────
SIM_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD      = os.path.join(SIM_DIR, "scene", "world.xml")
OUT_DIR    = os.path.join(SIM_DIR, "outputs")
FRAMES_DIR = os.path.join(OUT_DIR, "frames_A1")
VIDEO_PATH = os.path.join(OUT_DIR, "joint_A1_oscillation.mp4")
os.makedirs(FRAMES_DIR, exist_ok=True)

# ── Simulation parameters ─────────────────────────────────────────────────
SIM_DURATION = 4.0      # seconds of oscillation to record
VIDEO_FPS    = 30       # output video frame rate
AMPLITUDE    = math.radians(60)   # ±60°
FREQ         = 0.5      # Hz (one full cycle every 2 s)

# ── Load model ────────────────────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(WORLD)
data  = mujoco.MjData(model)
model.vis.global_.offwidth  = 1280
model.vis.global_.offheight = 720

act_a1  = model.actuator("act_a1").id
dt      = model.opt.timestep                       # 0.001 s
steps_per_frame = max(1, int(round(1.0 / (VIDEO_FPS * dt))))  # ~33 steps/frame
total_steps     = int(SIM_DURATION / dt)

print(f"Simulation: {SIM_DURATION}s at {1/dt:.0f} Hz physics")
print(f"Video:      {VIDEO_FPS} fps  →  1 frame every {steps_per_frame} steps")
print(f"Total frames to render: {total_steps // steps_per_frame}")

# ── Camera ────────────────────────────────────────────────────────────────
cam = mujoco.MjvCamera()
cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0.0, 0.1, 1.1]
cam.distance  = 2.8
cam.azimuth   = 140
cam.elevation = -18

# ── Render loop ───────────────────────────────────────────────────────────
frame_idx = 0
mujoco.mj_resetData(model, data)

with mujoco.Renderer(model, height=720, width=1280) as renderer:
    for step in range(total_steps):
        t = step * dt
        # Sinusoidal target: A * sin(2π f t)
        target = AMPLITUDE * math.sin(2 * math.pi * FREQ * t)
        data.ctrl[act_a1] = target
        mujoco.mj_step(model, data)

        if step % steps_per_frame == 0:
            renderer.update_scene(data, camera=cam)
            pixels = renderer.render()
            fname  = os.path.join(FRAMES_DIR, f"frame_{frame_idx:05d}.png")
            _save(pixels, fname)
            frame_idx += 1

            if frame_idx % 10 == 0:
                pct = 100 * step / total_steps
                print(f"  [{pct:5.1f}%] frame {frame_idx:4d}  t={t:.3f}s  A1={math.degrees(target):+.1f}°")

print(f"\nRendered {frame_idx} frames.")

# ── Encode with ffmpeg ────────────────────────────────────────────────────
ffmpeg_bin = shutil.which("ffmpeg")
if ffmpeg_bin is None:
    print("ERROR: ffmpeg not found. Install with: sudo apt install ffmpeg")
    print(f"Frames are saved in: {FRAMES_DIR}")
    raise SystemExit(1)

cmd = [
    ffmpeg_bin,
    "-y",                                           # overwrite output
    "-framerate", str(VIDEO_FPS),
    "-i", os.path.join(FRAMES_DIR, "frame_%05d.png"),
    "-c:v", "libx264",
    "-pix_fmt", "yuv420p",                          # max compatibility
    "-crf", "18",                                   # high quality
    VIDEO_PATH,
]
print(f"\nEncoding video: {' '.join(cmd)}")
result = subprocess.run(cmd, capture_output=True, text=True)

if result.returncode != 0:
    print("ffmpeg stderr:", result.stderr)
    raise RuntimeError("ffmpeg encoding failed")

print(f"\nVideo saved → {VIDEO_PATH}")

# ── Clean up frames ───────────────────────────────────────────────────────
shutil.rmtree(FRAMES_DIR)
print(f"Cleaned up frames directory.")
print("Task 3.9 PASS ✓")
