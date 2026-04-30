"""
Task 1.6 / 1.7 / 1.8 / 1.9 / 1.10 — KUKA KR6 R900 sixx arm loader.

Provides load_arm_model(urdf_path) as the single entry point for all scripts
that need the arm. Swap the arm in the future by changing this function only.

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


if __name__ == "__main__":
    model = load_arm_model()
    data = mujoco.MjData(model)

    # ── Task 1.8: print all joints ──
    print(f"\nJoints ({model.njnt} total):")
    for i in range(model.njnt):
        jnt = model.joint(i)
        jtype = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(jnt.type[0], str(jnt.type[0]))
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
    outputs_dir = os.path.join(os.path.dirname(__file__), "..", "outputs")
    outputs_dir = os.path.abspath(outputs_dir)
    os.makedirs(outputs_dir, exist_ok=True)
    out_path = os.path.join(outputs_dir, "arm_home_pose.png")

    # Set framebuffer size before creating Renderer (URDF <mujoco> tag is not picked up)
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 720

    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=-1)  # default free camera
        pixels = renderer.render()

    import PIL.Image
    PIL.Image.fromarray(pixels).save(out_path)
    print(f"\nTask 1.10: saved render → {out_path}")

    # ── Task 1.7: open passive viewer ──
    print("\nTask 1.7: Opening MuJoCo passive viewer (close window to exit)...")
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            mujoco.mj_step(model, data)
            v.sync()

    print("Viewer closed.")
