"""
Task 1.6 / 1.7 / 1.8 / 1.9 / 1.10 — KUKA KR6 R900 sixx arm loader.

Single entry point for loading the arm into MuJoCo.
All other scripts import load_arm_model() from here.
Swapping the arm in the future means changing this file only.

Public API:
    load_arm_model(urdf_path) -> MjModel
    render_to_png(model, data, path, camera, width, height) -> None
    apply_gains(model, gains, armature) -> None

Usage (Tasks 1.6–1.10):
    python -m arm.load_arm
"""
import math
import os
import mujoco
import mujoco.viewer
import numpy as np

_DEFAULT_URDF = os.path.join(
    os.path.dirname(__file__),
    "..", "assets", "urdf", "kr6r900sixx.urdf"
)
_DEFAULT_URDF = os.path.abspath(_DEFAULT_URDF)

REVOLUTE_JOINT_NAMES = [
    "joint_a1", "joint_a2", "joint_a3",
    "joint_a4", "joint_a5", "joint_a6",
]

# MuJoCo joint type integer → name mapping (used in diagnostics)
_JOINT_TYPE_NAMES = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}

# KUKA KR6 R900 sixx datasheet limits (degrees) for verification
_DATASHEET_DEG = {
    "joint_a1": (-170, 170),
    "joint_a2": (-190, 45),
    "joint_a3": (-120, 156),
    "joint_a4": (-185, 185),
    "joint_a5": (-120, 120),
    "joint_a6": (-350, 350),
}


def load_arm_model(urdf_path: str = _DEFAULT_URDF) -> mujoco.MjModel:
    """Load the KUKA KR6 URDF into MuJoCo and return the MjModel."""
    model = mujoco.MjModel.from_xml_path(urdf_path)
    print(f"Loaded: {urdf_path}")
    print(f"  nq (degrees of freedom): {model.nq}")
    return model


def apply_gains(
    model: mujoco.MjModel,
    gains: dict,
    armature: float = 0.5,
) -> None:
    """Set PD gains and rotor inertia on all actuators. Call once after model load.

    gains: {actuator_name: (kp, kv)}
    armature: virtual rotor inertia added to every DOF (prevents wrist instability)
    biasprm[1] must equal -gainprm[0] or position equilibrium shifts.
    """
    for name, (kp, kv) in gains.items():
        ai = model.actuator(name).id
        model.actuator_gainprm[ai, 0] =  kp
        model.actuator_biasprm[ai, 1] = -kp
        model.actuator_biasprm[ai, 2] = -kv
    for i in range(model.nv):
        model.dof_armature[i] = armature


def render_to_png(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    path: str,
    camera: int = -1,
    width: int = 1280,
    height: int = 720,
) -> None:
    """Render the current sim state offscreen and save to a PNG file."""
    import PIL.Image
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
    PIL.Image.fromarray(pixels).save(path)
    print(f"Saved render → {path}")


if __name__ == "__main__":
    model = load_arm_model()
    data = mujoco.MjData(model)

    # ── Task 1.8: print all joints ──
    print(f"\nJoints ({model.njnt} total):")
    for i in range(model.njnt):
        jnt = model.joint(i)
        jtype = _JOINT_TYPE_NAMES.get(jnt.type[0], str(jnt.type[0]))
        print(f"  [{i}] {jnt.name:30s}  type={jtype}")

    # ── Task 1.9: verify joint limits ──
    print("\nJoint limits (revolute joints):")
    for name in REVOLUTE_JOINT_NAMES:
        jnt = model.joint(name)
        lo_deg = math.degrees(model.jnt_range[jnt.id][0])
        hi_deg = math.degrees(model.jnt_range[jnt.id][1])
        ds_lo, ds_hi = _DATASHEET_DEG[name]
        match = "✓" if abs(lo_deg - ds_lo) < 2 and abs(hi_deg - ds_hi) < 2 else "✗"
        print(f"  {name}: [{lo_deg:+.1f}°, {hi_deg:+.1f}°]  datasheet [{ds_lo:+d}°, {ds_hi:+d}°]  {match}")

    # ── Task 1.10: offscreen render → outputs/arm_home_pose.png ──
    outputs_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "outputs"))
    os.makedirs(outputs_dir, exist_ok=True)
    render_to_png(model, data, os.path.join(outputs_dir, "arm_home_pose.png"))

    # ── Task 1.7: open passive viewer ──
    print("\nTask 1.7: Opening MuJoCo passive viewer (close window to exit)...")
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            mujoco.mj_step(model, data)
            v.sync()

    print("Viewer closed.")
