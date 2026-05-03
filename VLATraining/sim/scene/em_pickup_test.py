"""
scene/em_pickup_test.py
═══════════════════════════════════════════════════════════════════════════
Phase 6, Task 6.7 — Full EM pick-up cycle.

Timeline
────────
 0 – 1 s   Arm at home pose (EM activate → False, box stays on table)
 1 – 2 s   Arm descends to snap pose  →  EM activates (box locked flat)
 2 – 3 s   Arm lifts  →  box carried up
 3 – 4 s   Hold

Video saved to outputs/em_pickup_test.mp4
Run:  python -m scene.em_pickup_test
"""

import math
import pathlib
import subprocess

import mujoco
import numpy as np
from PIL import Image

from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate

# ─────────────────────────────────────────────────────────────────────────────
_GAINS = {
    "act_a1": (800., 80.), "act_a2": (800., 80.), "act_a3": (500., 50.),
    "act_a4": (300., 30.), "act_a5": (500., 50.), "act_a6": (300., 30.),
}

def _set_joints(data, model, angles_deg: dict):
    for jname, deg in angles_deg.items():
        data.ctrl[model.actuator("act_" + jname).id] = math.radians(deg)

def _render(renderer, model, data, cam):
    renderer.update_scene(data, camera=cam)
    return Image.fromarray(renderer.render())

# ─────────────────────────────────────────────────────────────────────────────
HOME_POSE    = {"a1": 0, "a2":   0, "a3":  0, "a4": 0, "a5":  0, "a6": 0}
ALMOST_THERE = {"a1": 0, "a2": -55, "a3": 80, "a4": 0, "a5": 45, "a6": 0}
SNAP_POSE    = {"a1": 0, "a2": -55, "a3": 90, "a4": 0, "a5": 55, "a6": 0}  # EM face points -Z
LIFT_POSE    = {"a1": 0, "a2": -65, "a3": 90, "a4": 0, "a5": 55, "a6": 0}

# ─────────────────────────────────────────────────────────────────────────────
def main():
    sim_dir   = pathlib.Path(__file__).parent
    out_dir   = sim_dir.parent / "outputs";   out_dir.mkdir(exist_ok=True)
    frames_dir = out_dir / "_em_pickup_frames"; frames_dir.mkdir(exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(sim_dir / "world.xml"))
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    # Find IDs
    em_site_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,  "em_contact_site")
    box_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,  "box_top_site")
    box_bid     = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,  "metal_box")
    box_jnt_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_freejoint")
    box_dof_adr = model.jnt_dofadr[box_jnt_id]
    adr         = model.jnt_qposadr[box_jnt_id]

    # ── Find snap-pose EM position ───────────────────────────────────
    _set_joints(data, model, SNAP_POSE)
    for _ in range(3000):
        mujoco.mj_step(model, data)
    em_snap = data.site_xpos[em_site_id].copy()
    print(f"EM pad in snap pose: {em_snap.round(4)}")

    # ── Reset and place box under EM (top face 3 cm below EM site) ──
    mujoco.mj_resetData(model, data)
    box_z = em_snap[2] - 0.05 - 0.015  # EM to box-top gap: 1.5 cm (within 2 cm threshold)
    data.qpos[adr:adr+3]   = [em_snap[0], em_snap[1], box_z]
    data.qpos[adr+3:adr+7] = [1, 0, 0, 0]   # upright
    mujoco.mj_forward(model, data)
    box_init_qpos = data.qpos[adr:adr+7].copy()
    print(f"Box top Z: {data.xpos[box_bid][2]+0.05:.4f}  |  EM Z (snap): {em_snap[2]:.4f}")

    # Camera
    cam = mujoco.MjvCamera()
    cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [em_snap[0], em_snap[1] - 0.2, em_snap[2] - 0.15]
    cam.distance  = 1.6; cam.azimuth = 200; cam.elevation = -20

    renderer  = mujoco.Renderer(model, height=720, width=1280)
    DT        = model.opt.timestep
    FPS       = 30
    EVERY     = max(1, int(round(1.0 / (FPS * DT))))
    frame_idx = 0
    em_active = False
    activated_at = None

    def save_frame():
        nonlocal frame_idx
        _render(renderer, model, data, cam).save(frames_dir / f"frame_{frame_idx:05d}.png")
        frame_idx += 1

    # ── Phase 0–1 s: home pose (confirm EM does NOT activate) ───────
    print("Phase 0→1 s: home pose …")
    _set_joints(data, model, HOME_POSE)
    for step in range(1000):
        data.qpos[adr:adr+7] = box_init_qpos
        data.qvel[box_dof_adr:box_dof_adr+6] = 0
        mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        if step % EVERY == 0: save_frame()
    activated = em_activate(model, data)
    print(f"  em_activate at home pose → {activated}  (expected False)")

    # ── Phase 1–2 s: descend to snap pose (EM activates) ────────────
    print("Phase 1→2 s: descending to snap pose …")
    _set_joints(data, model, SNAP_POSE)
    for step in range(1000):
        # Box stays still until EM locks
        if not em_active:
            data.qpos[adr:adr+7] = box_init_qpos
            data.qvel[box_dof_adr:box_dof_adr+6] = 0
            mujoco.mj_forward(model, data)
        mujoco.mj_step(model, data)
        if not em_active and em_activate(model, data):
            em_active    = True
            activated_at = step * DT + 1.0
            dist = float(np.linalg.norm(
                data.site_xpos[em_site_id] - data.site_xpos[box_site_id]))
            print(f"  *** EM ACTIVATED at t={activated_at:.3f} s  gap={dist*100:.1f} cm ***")
        if step % EVERY == 0: save_frame()

    if not em_active:
        print("WARNING: EM did not activate — check poses/threshold")

    # ── Phase 2–3 s: lift ────────────────────────────────────────────
    print("Phase 2→3 s: lifting …")
    _set_joints(data, model, LIFT_POSE)
    box_z_before = float(data.xpos[box_bid][2])
    for step in range(1000):
        mujoco.mj_step(model, data)
        if step % EVERY == 0: save_frame()
    box_z_after = float(data.xpos[box_bid][2])
    lift = box_z_after - box_z_before
    print(f"  Box Z: {box_z_before:.4f} → {box_z_after:.4f}  (lifted {lift*100:.1f} cm)")

    # ── Phase 3–4 s: hold ────────────────────────────────────────────
    print("Phase 3→4 s: holding …")
    for step in range(1000):
        mujoco.mj_step(model, data)
        if step % EVERY == 0: save_frame()

    print(f"\nTotal frames: {frame_idx}")

    # ── Encode video ─────────────────────────────────────────────────
    out_mp4 = out_dir / "em_pickup_test.mp4"
    subprocess.run([
        "ffmpeg", "-y", "-framerate", str(FPS),
        "-i", str(frames_dir / "frame_%05d.png"),
        "-vf", "scale=1280:720",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out_mp4)
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"Video → {out_mp4}  ({out_mp4.stat().st_size//1024} KB)")

    print("\n══ RESULT ══════════════════════════════════")
    print(f"  EM activated : {'YES at t=' + f'{activated_at:.3f}s' if em_active else 'NO'}")
    print(f"  Box lifted   : {lift*100:.1f} cm")
    if em_active and lift > 0.02:
        print("  PASS ✓")
    else:
        print("  FAIL ✗")
    print("════════════════════════════════════════════")

if __name__ == "__main__":
    main()
