"""
VLATraining/sim/tools/vla_live_viewer.py  —  Phase 10/11
═══════════════════════════════════════════════════════════════════════════════
Live real-time closed-loop VLA execution with interactive 3D MuJoCo viewer.
Loads local JAX-based Octo model and runs full pick-and-place task tree.

Prerequisites:
  • Conda env `vla` activated
  • Running on a system with a display server (Michael's Linux Desktop)

Run:
  /home/michael/miniforge3/envs/vla/bin/python3 \\
      ~/Desktop/VLA_Tutorial/VLATraining/sim/tools/vla_live_viewer.py
"""

import os
import sys
import time
import pathlib
import numpy as np
import mujoco
import mujoco.viewer
import jax

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

# ML imports
from octo.model.octo_model import OctoModel

# Constants
WORLD_XML = str(_SIM_DIR / "scene" / "world.xml")
CAM_OVERHEAD = "cam_overhead"
CAM_WRIST = "cam_wrist"

# PD Gains
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


def main():
    print("════════════════════════════════════════════════════════")
    print("  Live Real-Time Closed-Loop VLA Simulation with Octo")
    print("════════════════════════════════════════════════════════")

    # 1. Initialize simulation environment
    print("[1/5] Loading MuJoCo scene XML and applying PD gains ...")
    model = mujoco.MjModel.from_xml_path(WORLD_XML)
    data  = mujoco.MjData(model)
    apply_gains(model, _GAINS)

    jids    = [model.joint(n).id for n in _JOINT_NAMES]
    qpas    = [model.jnt_qposadr[j] for j in jids]
    act_ids = [model.actuator(n).id for n in _ACT_NAMES]

    box_jid = model.joint("box_freejoint").id
    box_qpa = model.jnt_qposadr[box_jid]

    mujoco.mj_resetData(model, data)
    apply_gains(model, _GAINS)

    # Position metal box at starting coordinates
    BOX_HALF = 0.05
    box_init = np.array([0.76, 0.0, TABLE_Z + BOX_HALF, 1.0, 0.0, 0.0, 0.0])
    data.qpos[box_qpa:box_qpa + 7] = box_init

    # Sync controllers
    for qi, ai in zip(qpas, act_ids):
        data.ctrl[ai] = data.qpos[qi]
    mujoco.mj_forward(model, data)

    # Settle arm at home
    print("[2/5] Settling robot arm at starting home pose ...")
    for _ in range(1000):
        data.qpos[box_qpa:box_qpa + 7] = box_init
        data.qvel[model.jnt_dofadr[box_jid]:model.jnt_dofadr[box_jid] + 6] = 0.0
        mujoco.mj_step(model, data)

    # 2. Setup Meta-Controller and Adapter
    nodes = build_pick_and_place_tree()
    manager = TaskTreeManager(nodes)
    adapter = OctoAdapter()
    adapter.set_current_pos(data.qpos[qpas].copy())

    # 3. Load VLA Model
    print("[3/5] Loading pretrained Octo model ('hf://rail-berkeley/octo-small-1.5') on GPU ...")
    octo_model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
    print(f"      Model loaded successfully on: {jax.devices()}")

    # JAX Warmup
    print("[4/5] Pre-compiling JAX JIT graph (warm-up step) ...")
    t_warm = time.monotonic()
    fake_image = np.zeros((1, 1, 256, 256, 3), dtype=np.uint8)
    dummy_obs = {
        "image_primary": fake_image,
        "pad_mask": np.array([[True]]),
        "timestep_pad_mask": np.array([[True]])
    }
    dummy_task = octo_model.create_tasks(texts=["locate the metal box on the table"])
    _ = octo_model.sample_actions(dummy_obs, dummy_task, rng=jax.random.PRNGKey(42))
    print(f"      JAX warm-up complete in {time.monotonic() - t_warm:.2f} s ✅")

    # 4. Launch Passive Viewer
    print("[5/5] Launching MuJoCo passive 3D viewer on your desktop ...")
    viewer = mujoco.viewer.launch_passive(model, data)
    if viewer is None or not viewer.is_running():
        print("[ERROR] Could not start MuJoCo viewer window. Make sure you are running inside a graphical environment.")
        sys.exit(1)

    # Align camera view in the window to center on table
    viewer.cam.lookat = [0.65, 0.1, 0.9]
    viewer.cam.distance = 1.3
    viewer.cam.elevation = -35
    viewer.cam.azimuth = 135

    print("\n🚀 Starting Real-Time Closed-Loop VLA Execution Loop!")
    print("Watch the 3D MuJoCo window on your screen to see the KUKA arm move live.\n")

    rng_key = jax.random.PRNGKey(42)
    step_idx = 0
    last_node_name = ""

    # headless renderer for feeding image observations to Octo
    ren_overhead = mujoco.Renderer(model, height=256, width=256)

    try:
        while viewer.is_running() and not manager.is_complete() and not manager.is_failed():
            t0 = time.monotonic()

            status_dict = manager.get_status_dict()
            active_node = status_dict["active_node"]
            instruction = status_dict["instruction"]

            if active_node != last_node_name:
                print(f"\n▶ SUBTASK TRANSITION: {active_node} — '{instruction}'")
                last_node_name = active_node

            # A. Visual observation capture (Bird's eye view camera)
            ren_overhead.update_scene(data, camera=CAM_OVERHEAD)
            overhead_rgb = ren_overhead.render().copy()

            # B. Query VLA Model
            task = octo_model.create_tasks(texts=[instruction])
            image_observation = np.expand_dims(np.expand_dims(overhead_rgb, axis=0), axis=0)
            observation = {
                "image_primary": image_observation,
                "pad_mask": np.array([[True]]),
                "timestep_pad_mask": np.array([[True]])
            }

            rng_key, subkey = jax.random.split(rng_key)
            actions = octo_model.sample_actions(observation, task, rng=subkey)
            action = np.asarray(actions[0, 0], dtype=np.float64)

            # Safety scaling
            action[:6] = action[:6] * 0.05

            # Translate deltas to targets
            cmd_dict = adapter.actions_to_joint_cmd(action)
            target_qpos = cmd_dict["joint_positions"]
            em_command  = cmd_dict["em_command"]

            # Set controller joint targets
            for ai, val in zip(act_ids, target_qpos):
                data.ctrl[ai] = val

            # EM snap / weld activation logic
            weld_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "em_weld")
            if active_node == "GRASP":
                if em_command and not data.eq_active[weld_id]:
                    data.qvel[:] = 0.0
                    em_activate(model, data, threshold_m=0.05)
                    print("  [GRASP] Activated Electromagnetic Weld constraint!")
            elif active_node == "PLACE":
                if not em_command and data.eq_active[weld_id]:
                    em_deactivate(model, data)
                    print("  [PLACE] Deactivated Electromagnetic Weld constraint — dropping box!")

            # C. Step physics smoothly (16 iterations at 6 Hz VLA control rate)
            # At each simulation step, we sleep and sync to get smooth 1x speed real-time render
            for _ in range(16):
                # Force pin box pre-grasp to prevent physics drifting
                if active_node in ["SEARCH", "APPROACH"]:
                    data.qpos[box_qpa:box_qpa + 7] = box_init
                    data.qvel[model.jnt_dofadr[box_jid]:model.jnt_dofadr[box_jid] + 6] = 0.0
                mujoco.mj_step(model, data)
                
                # Smooth 1x real-time sync (1 ms physics timestep -> sleep 1 ms)
                time.sleep(0.001)
                viewer.sync()

            # D. Evaluate Task Tree conditions
            reading = SensorReading.from_mujoco(model, data)
            manager.step(reading)

            # Log status printout every VLA step (~160 ms)
            dt_step = time.monotonic() - t0
            print(f"  VLA step {step_idx:<3d} | elapsed={dt_step*1000.0:+.1f} ms | prox={reading.proximity:.3f} m | ee_fz={reading.ee_force[2]:+.2f} N | em={'ON' if reading.em_active else 'OFF'}")
            step_idx += 1

        print("\n════════════════════════════════════════════════════════")
        print("  VLA RUN COMPLETED SUMMARY")
        print("════════════════════════════════════════════════════════")
        print(f"  Completed successfully : {manager.is_complete()}")
        print(f"  Failed on timeout      : {manager.is_failed()}")
        print(f"  Total VLA Steps        : {step_idx} steps")
        print("════════════════════════════════════════════════════════")

    except KeyboardInterrupt:
        print("\nLive VLA run interrupted by user.")
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
