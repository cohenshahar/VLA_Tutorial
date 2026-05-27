"""
VLATraining/sim/tools/interactive_vla_step.py  —  Phase 10/11
═══════════════════════════════════════════════════════════════════════════════
Stateful interactive step-by-step runner for closed-loop VLA control in MuJoCo.
Saves and loads simulation and Task Tree states to/from a binary .npz file.

Features:
  1. Standalone execution without ROS2.
  2. Lazy imports of JAX & Octo to keep initialization instant (< 0.5s).
  3. Seamlessly tracks and serializes MuJoCo MjData, MjModel equality data,
     OctoAdapter joint command history, and TaskTreeManager operational nodes.
  4. Saves composite visual frames (overhead + wrist view) with clear overlays.
"""

import os
import sys
import argparse
import pathlib
import numpy as np
import cv2
import mujoco

# Path bootstrap to allow relative/absolute imports
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from arm.load_arm import apply_gains
from scene.em_controller import em_activate, em_deactivate
from task_tree.sensor_reading import SensorReading
from task_tree.task_node import TaskNode
from task_tree.pick_and_place_tree import build_pick_and_place_tree, TABLE_Z, TARGET_XY
from task_tree.task_tree_manager import TaskTreeManager
from tools.octo_adapter import OctoAdapter

# Constants
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
CAM_WRIST = "cam_wrist"
CAM_OVERHEAD = "cam_overhead"

PANEL_W, PANEL_H = 640, 720
RENDER_SKIP = 16

# PD Gains mapping identical to scripted test
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

# Text overlay helper (identical to test_task_tree)
def _overlay_text(frame_rgb, lines):
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


def do_init(state_path, img_path):
    print("[INIT] Initializing MuJoCo simulation environment ...")
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids    = [model.joint(n).id for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j] for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]

    # Reset data
    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)

    # Place box at table starting center
    BOX_HALF = 0.05
    box_init = np.array([0.76, 0.0, TABLE_Z + BOX_HALF, 1.0, 0.0, 0.0, 0.0])
    data.qpos[box_qpa:box_qpa + 7] = box_init

    # Sync PD target controllers
    for qi, ai in zip(qpas, act_ids):
        data.ctrl[ai] = data.qpos[qi]

    mujoco.mj_forward(model, data)

    # Settle arm at home pose for 1000 steps to damp initial movements
    print("[INIT] Settling arm at Home pose ...")
    for _ in range(1000):
        # Force pin box during arm settling
        data.qpos[box_qpa:box_qpa + 7] = box_init
        data.qvel[model.jnt_dofadr[box_jid]:model.jnt_dofadr[box_jid] + 6] = 0.0
        mujoco.mj_step(model, data)

    # Setup TaskTreeManager and OctoAdapter
    nodes = build_pick_and_place_tree()
    manager = TaskTreeManager(nodes)
    adapter = OctoAdapter()
    adapter.set_current_pos(data.qpos[qpas].copy())

    # Pre-warm JAX key generator arrays (standard RNG state serialization)
    jax_rng_key = np.array([0, 0], dtype=np.uint32)

    # Save state
    np.savez(
        state_path,
        qpos=data.qpos,
        qvel=data.qvel,
        ctrl=data.ctrl,
        act=data.act if data.act is not None else np.array([]),
        time=np.array([data.time]),
        eq_active=data.eq_active,
        eq_active0=model.eq_active0,
        eq_data=model.eq_data,
        adapter_qpos=adapter._qpos,
        jax_rng=jax_rng_key,
        active_idx=np.array([manager._idx]),
        node_steps=np.array([n.steps_active for n in manager._nodes]),
        node_statuses=np.array([n.status for n in manager._nodes])
    )
    print(f"[INIT] Simulation state saved successfully to {state_path} ✅")

    # Render starting frame
    render_and_overlay(model, data, manager, img_path, "[READY] Ready to begin close-loop execution")


def render_and_overlay(model, data, manager, img_path, additional_text=None):
    ren_wrist    = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)
    ren_overhead = mujoco.Renderer(model, height=PANEL_H, width=PANEL_W)

    reading = SensorReading.from_mujoco(model, data)
    status_dict = manager.get_status_dict()

    lines = [
        f"Active Subtask: {status_dict['active_node']}",
        f"Instruction:    '{status_dict['instruction']}'",
        f"Sensor Prox:    {reading.proximity:.4f} m",
        f"EE Load Fz:     {reading.ee_force[2]:+.2f} N",
        f"Weld Active:    {'YES' if reading.em_active else 'NO'}",
        f"Target Touch:   {'YES' if reading.target_contact > 0.0 else 'NO'}",
    ]
    if additional_text:
        lines.append(f"State Details:  {additional_text}")

    # Render overhead camera
    ren_overhead.update_scene(data, camera=CAM_OVERHEAD)
    right = ren_overhead.render().copy()

    # Render wrist camera
    ren_wrist.update_scene(data, camera=CAM_WRIST)
    left = ren_wrist.render().copy()

    # Add text overlay to left (wrist camera panel)
    left = _overlay_text(left, lines)

    # Stitch panels
    frame = np.concatenate([left, right], axis=1)

    # Save PNG
    pathlib.Path(img_path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(img_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    print(f"[RENDER] Diagnostic frame saved to {img_path} 🖼️")


def do_step(state_path, img_path):
    print("[STEP] Loading state and lazy-importing ML libraries ...")
    if not os.path.exists(state_path):
        print(f"[ERROR] State file '{state_path}' not found! Run --init first.")
        sys.exit(1)

    # 1. Load ML packages
    try:
        import jax
        from octo.model.octo_model import OctoModel
    except ImportError as exc:
        print(f"[ERROR] Required ML packages not found. Run this inside 'vla' conda environment: {exc}")
        sys.exit(1)

    # 2. Re-instantiate model & data
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids    = [model.joint(n).id for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j] for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]

    # Load saved state
    state = np.load(state_path)
    data.qpos[:] = state['qpos']
    data.qvel[:] = state['qvel']
    data.ctrl[:] = state['ctrl']
    if len(state['act']) > 0:
        data.act[:] = state['act']
    data.time = float(state['time'][0])
    data.eq_active[:] = state['eq_active']
    model.eq_active0[:] = state['eq_active0']
    model.eq_data[:] = state['eq_data']

    # Load Adapter
    adapter = OctoAdapter()
    adapter.set_current_pos(state['adapter_qpos'])

    # Load TaskTreeManager
    nodes = build_pick_and_place_tree()
    manager = TaskTreeManager(nodes)
    manager._idx = int(state['active_idx'][0])
    for idx, node in enumerate(manager._nodes):
        node.steps_active = int(state['node_steps'][idx])
        node.status = str(state['node_statuses'][idx])

    # Settle RNG key
    jax_rng_key = state['jax_rng']
    rng_key = jax.random.PRNGKey(int(jax_rng_key[0]))
    if jax_rng_key[0] == 0:
        # Re-seed on step 0
        rng_key = jax.random.PRNGKey(42)

    status_dict = manager.get_status_dict()
    active_node = status_dict["active_node"]
    instruction = status_dict["instruction"]

    print(f"\n▶ CURRENT ACTIVE SUBTASK: {active_node} ('{instruction}')")

    # 3. Visual observation capture & VLA model query
    print("[STEP] Generating bird's-eye visual observation from overhead camera ...")
    ren_overhead = mujoco.Renderer(model, height=256, width=256)
    ren_overhead.update_scene(data, camera=CAM_OVERHEAD)
    overhead_rgb = ren_overhead.render().copy()

    # Model inference
    print("[STEP] Querying GPU JAX Octo model ...")
    octo_model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    task = octo_model.create_tasks(texts=[instruction])

    # Add temporal history and batch dims
    image_observation = np.expand_dims(np.expand_dims(overhead_rgb, axis=0), axis=0)
    observation = {
        "image_primary": image_observation,
        "pad_mask": np.array([[True]]),
        "timestep_pad_mask": np.array([[True]])
    }

    # Split RNG
    rng_key, subkey = jax.random.split(rng_key)
    actions = octo_model.sample_actions(observation, task, rng=subkey)
    action = np.asarray(actions[0, 0], dtype=np.float64)

    # Apply safety multiplier
    action[:6] = action[:6] * 0.05
    print(f"[STEP] Predicted raw deltas (scaled): {np.round(action[:6], 4)}")

    # Feed into Adapter
    cmd_dict = adapter.actions_to_joint_cmd(action)
    target_qpos = cmd_dict["joint_positions"]
    em_command  = cmd_dict["em_command"]

    print(f"[STEP] Target joint positions: {np.round(target_qpos, 3)}")
    print(f"[STEP] Target EM Gripper State: {'ACTIVE' if em_command else 'INACTIVE'}")

    # Set controller target positions
    for ai, val in zip(act_ids, target_qpos):
        data.ctrl[ai] = val

    # EM Controller state changes
    if active_node == "GRASP":
        if em_command and not data.eq_active[model.eq_equality("em_weld").id]:
            # Zero velocities first to snap clean
            data.qvel[:] = 0.0
            em_activate(model, data, threshold_m=0.05)
            print("[GRASP] Snap box and activate weld constraint!")
    elif active_node == "PLACE":
        if not em_command and data.eq_active[model.eq_equality("em_weld").id]:
            em_deactivate(model, data)
            print("[PLACE] Deactivate weld constraint - releasing box!")

    # 4. Advance physics by 16 simulation steps (representing 6 Hz VLA control interval)
    print(f"[STEP] Stepping MuJoCo physics for {RENDER_SKIP} iterations ...")
    for _ in range(RENDER_SKIP):
        # Force pin box pre-grasp to prevent drifting due to micro-vibrations
        if active_node in ["SEARCH", "APPROACH"]:
            box_init_xy = np.array([0.76, 0.0, TABLE_Z + 0.05, 1.0, 0.0, 0.0, 0.0])
            data.qpos[box_qpa:box_qpa + 7] = box_init_xy
            data.qvel[model.jnt_dofadr[box_jid]:model.jnt_dofadr[box_jid] + 6] = 0.0
        mujoco.mj_step(model, data)

    # 5. Read sensors and tick TaskTreeManager
    reading = SensorReading.from_mujoco(model, data)
    manager.step(reading)

    # Print transitioned state if any
    status_dict_after = manager.get_status_dict()
    if status_dict_after["active_node"] != active_node:
        print(f"[TRANSITION] Subtask completed! Transitioning to: {status_dict_after['active_node']} 🎉")

    # Re-serialize PRNGKey state
    jax_rng_state = np.array([rng_key[0], rng_key[1]], dtype=np.uint32)

    # Save state back
    np.savez(
        state_path,
        qpos=data.qpos,
        qvel=data.qvel,
        ctrl=data.ctrl,
        act=data.act if data.act is not None else np.array([]),
        time=np.array([data.time]),
        eq_active=data.eq_active,
        eq_active0=model.eq_active0,
        eq_data=model.eq_data,
        adapter_qpos=adapter._qpos,
        jax_rng=jax_rng_state,
        active_idx=np.array([manager._idx]),
        node_steps=np.array([n.steps_active for n in manager._nodes]),
        node_statuses=np.array([n.status for n in manager._nodes])
    )
    print("[STEP] State saved successfully ✅")

    # Render frame
    step_info = f"Joint targets set. EM={'ON' if em_command else 'OFF'}"
    render_and_overlay(model, data, manager, img_path, step_info)


def main():
    parser = argparse.ArgumentParser(description="Stateful VLA Step Runner")
    parser.add_argument("--init", action="store_true", help="Initialize simulation and Task Tree")
    parser.add_argument("--step", action="store_true", help="Run a single VLA inference and physics step")
    parser.add_argument("--state-file", type=str, required=True, help="Path to serialize npz simulation state")
    parser.add_argument("--image-file", type=str, required=True, help="Path to render the composite visual frame")

    args = parser.parse_args()

    if args.init:
        do_init(args.state_file, args.image_file)
    elif args.step:
        do_step(args.state_file, args.image_file)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
