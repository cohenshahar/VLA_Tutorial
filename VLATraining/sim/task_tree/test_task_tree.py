"""
sim/task_tree/test_task_tree.py  —  Phase 10, Task 10.14

Integration test running a full pick-and-place sequence on the KUKA arm,
driven by scripted controllers but transitioned entirely by the TaskTreeManager.
Generates a side-by-side overhead + wrist camera video with live diagnostics overlay.
"""

import os
import sys
import math
import pathlib
from datetime import datetime

import imageio
import cv2
import mujoco
import numpy as np

# Path bootstrap to allow relative/absolute imports
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate
from scene._ik_utils import R_DOWN, _run_ik, _sync_ctrl, _a1_toward

from task_tree.sensor_reading import SensorReading
from task_tree.task_node import TaskNode
from task_tree.pick_and_place_tree import build_pick_and_place_tree, TABLE_Z, TARGET_XY
from task_tree.task_tree_manager import TaskTreeManager

# ── I/O ───────────────────────────────────────────────────────────────────────
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
_DATE     = datetime.now().strftime("%Y%m%d")
OUT_VIDEO = str(_SIM_DIR / "outputs" / f"task_tree_test_{_DATE}.mp4")

# ── Scene geometry ────────────────────────────────────────────────────────────
BOX_HALF   = 0.05
BOX_POS    = np.array([0.76, 0.0, TABLE_Z + BOX_HALF])
ARM_BASE   = np.array([0.0, -0.3])

APPROACH_Z = TABLE_Z + 3.0 * (2 * BOX_HALF) + 0.05   # 1.20 m
SNAP_Z     = BOX_POS[2] + BOX_HALF + 0.015            # 0.965 m
LIFT_Z     = SNAP_Z + 0.15                            # 1.115 m
PLACE_EE_Z = 0.86 + 0.02 + 2 * BOX_HALF               # 0.98 m (2 cm drop)

# ── Video layout ──────────────────────────────────────────────────────────────
PANEL_W, PANEL_H = 640, 720
VID_W = PANEL_W * 2
VID_H = PANEL_H
RENDER_SKIP = 16

CAM_WRIST    = "cam_wrist"
CAM_OVERHEAD = "cam_overhead"

# ── PD gains ──────────────────────────────────────────────────────────────────
_GAINS = {
    "act_a1": (800., 80.),
    "act_a2": (800., 80.),
    "act_a3": (500., 50.),
    "act_a4": (300., 30.),
    "act_a5": (500., 50.),
    "act_a6": (300., 30.),
}

_JOINT_NAMES = [f"joint_a{i}" for i in range(1, 7)]
_ACT_NAMES   = [f"act_a{i}"   for i in range(1, 7)]


# ── Text overlay helper ───────────────────────────────────────────────────────
def _overlay_text(frame_rgb, lines):
    # cv2 works in BGR
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    font       = cv2.FONT_HERSHEY_SIMPLEX
    scale      = 0.65
    thickness  = 2
    color_text = (255, 255, 64)    # yellow
    color_bg   = (0, 0, 0)         # black shadow

    for i, line in enumerate(lines):
        x, y = 10, 28 + i * 28
        # Drop shadow
        cv2.putText(frame_bgr, line, (x + 1, y + 1), font, scale, color_bg, thickness, cv2.LINE_AA)
        cv2.putText(frame_bgr, line, (x, y), font, scale, color_text, thickness, cv2.LINE_AA)
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)


def main():
    # 1. Load model and data
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids    = [model.joint(n).id for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j] for j in jids]
    dofas   = [model.jnt_dofadr[j]  for j in jids]
    ranges  = [tuple(model.jnt_range[j]) for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]
    site_id = model.site("em_contact_site").id

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]
    box_doa = model.jnt_dofadr[box_jid]

    # 2. Build task tree & manager
    nodes = build_pick_and_place_tree()
    manager = TaskTreeManager(nodes)

    # 3. Setup video writer
    pathlib.Path(OUT_VIDEO).parent.mkdir(parents=True, exist_ok=True)
    fps    = max(1, round(1.0 / (model.opt.timestep * RENDER_SKIP)))
    writer = imageio.get_writer(OUT_VIDEO, fps=fps)

    ren_wrist    = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)
    ren_overhead = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)

    # 4. Settle at Home first (pre-task-tree phase)
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)
    box_init = data.qpos[box_qpa:box_qpa + 7].copy()
    _sync_ctrl(data, qpas, act_ids)
    mujoco.mj_forward(model, data)

    print("[1] Settling at home …")
    for _ in range(1000):
        mujoco.mj_step(model, data)

    # Pre-calculated alignment targets
    a1_box_align = _a1_toward(BOX_POS, ARM_BASE, ranges)
    a1_tgt_align = _a1_toward(TARGET_XY, ARM_BASE, ranges)

    step_idx = 0
    last_node_name = ""

    # Gripper trigger checks
    grasp_activated = False
    place_deactivated = False

    print("\nStarting Task Tree driven Pick-and-Place sequence!")

    while not manager.is_complete() and not manager.is_failed():
        # A. Get current active node details
        status_dict = manager.get_status_dict()
        active_node = status_dict["active_node"]
        steps_active = status_dict["steps_active"]

        if active_node != last_node_name:
            print(f"\n▶ Subtask Transition: {active_node} — '{status_dict['instruction']}'")
            last_node_name = active_node

        # B. Apply Trajectory control based on active node and active steps
        if active_node == "SEARCH":
            # Search is purely visual/observational, passes instantly.
            pass

        elif active_node == "APPROACH":
            if steps_active < 2000:
                # Pre-align A1 toward box
                data.qpos[qpas[0]]    = a1_box_align
                data.ctrl[act_ids[0]] = a1_box_align
            elif steps_active < 3000:
                # Rise to clearance height
                ee_xy = data.site_xpos[site_id][:2].copy()
                _run_ik(model, data, dofas, qpas, ranges, site_id,
                          np.array([ee_xy[0], ee_xy[1], APPROACH_Z]), R_DOWN)
                _sync_ctrl(data, qpas, act_ids)
            elif steps_active < 4500:
                # Translate over box
                _run_ik(model, data, dofas, qpas, ranges, site_id,
                          np.array([BOX_POS[0], BOX_POS[1], APPROACH_Z]), R_DOWN)
                _sync_ctrl(data, qpas, act_ids)
            else:
                # Descend to box snap height (pin box to prevent physics drift)
                _run_ik(model, data, dofas, qpas, ranges, site_id,
                          np.array([BOX_POS[0], BOX_POS[1], SNAP_Z]), R_DOWN, pos_tol=3e-3)
                _sync_ctrl(data, qpas, act_ids)
                data.qpos[box_qpa:box_qpa + 7] = box_init
                data.qvel[box_doa:box_doa + 6] = 0.0

        elif active_node == "GRASP":
            if not grasp_activated:
                data.qvel[:] = 0.0
                data.qpos[box_qpa:box_qpa + 7] = box_init
                mujoco.mj_forward(model, data)
                em_activate(model, data, threshold_m=0.05)
                grasp_activated = True
                print("  [GRASP] Activated EM pad weld constraint")

        elif active_node == "LIFT":
            # Lift to carry clearance Z
            ee_xy_now = data.site_xpos[site_id][:2].copy()
            _run_ik(model, data, dofas, qpas, ranges, site_id,
                      np.array([ee_xy_now[0], ee_xy_now[1], LIFT_Z]))
            _sync_ctrl(data, qpas, act_ids)

        elif active_node == "LOCATE_TARGET":
            # Locate target is purely visual/observational, passes instantly.
            pass

        elif active_node == "TRANSPORT":
            if steps_active < 2000:
                # Pre-align A1 toward target zone
                data.qpos[qpas[0]]    = a1_tgt_align
                data.ctrl[act_ids[0]] = a1_tgt_align
            elif steps_active < 3500:
                # Translate over target zone
                _run_ik(model, data, dofas, qpas, ranges, site_id,
                          np.array([TARGET_XY[0], TARGET_XY[1], LIFT_Z]), max_iter=800)
                _sync_ctrl(data, qpas, act_ids)
            else:
                # Descend to placement height (2 cm above target surface)
                _run_ik(model, data, dofas, qpas, ranges, site_id,
                          np.array([TARGET_XY[0], TARGET_XY[1], PLACE_EE_Z]), pos_tol=3e-3)
                _sync_ctrl(data, qpas, act_ids)

        elif active_node == "PLACE":
            if not place_deactivated:
                em_deactivate(model, data)
                place_deactivated = True
                print("  [PLACE] Deactivated EM pad weld constraint — dropping box")

        # C. Advance physics step
        mujoco.mj_step(model, data)

        # D. Read sensors and update Task Tree Manager
        reading = SensorReading.from_mujoco(model, data)
        manager.step(reading)

        # E. Record video frames at RENDER_SKIP intervals
        if step_idx % RENDER_SKIP == 0:
            status_dict = manager.get_status_dict()
            lines = [
                f"Subtask: {status_dict['active_node']}",
                f"Status:  {status_dict['node_statuses'][status_dict['active_idx']]['status']}",
                f"Prox:    {reading.proximity:.4f} m",
                f"EE Fz:   {reading.ee_force[2]:+.2f} N",
                f"EM:      {'ON' if reading.em_active else 'OFF'}",
                f"Contact: {'YES' if reading.target_contact > 0.0 else 'NO'}",
            ]
            ren_wrist.update_scene(data, camera=CAM_WRIST)
            left = ren_wrist.render().copy()

            ren_overhead.update_scene(data, camera=CAM_OVERHEAD)
            right = ren_overhead.render().copy()

            left  = _overlay_text(left, lines)
            frame = np.concatenate([left, right], axis=1)
            writer.append_data(frame)

        step_idx += 1

    # Close video
    writer.close()

    # 5. Verify outcome
    print("\n══ TASK TREE EXECUTION SUMMARY ═════════════")
    status_dict = manager.get_status_dict()
    print(f"  Final Node  : {status_dict['active_node']}")
    print(f"  Complete    : {manager.is_complete()}")
    print(f"  Failed      : {manager.is_failed()}")
    print(f"  Total Steps : {step_idx} steps")
    print(f"  Video saved : {OUT_VIDEO}")
    
    assert manager.is_complete(), "Task Tree pick-and-place failed to complete!"
    print("  RESULT      : PASS ✅")
    print("════════════════════════════════════════════")


if __name__ == "__main__":
    main()
