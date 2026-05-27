"""
tools/real_vla_loop.py — Phase 10 / 11
═══════════════════════════════════════════════════════════════════════════════
Closed-loop local real VLA inference loop. Runs real-time local Octo inference
using the GPU and the vla conda environment.

Pipeline (6 Hz):
  /camera/wrist/image_raw  (ROS2 sensor_msgs/Image)
       ↓
  Local JAX Octo Model     (Inference on GPU)
       ↓
  7-float action           [Δj1, Δj2, Δj3, Δj4, Δj5, Δj6, gripper]
       ↓
  OctoAdapter              (Accumulates deltas and clips limits)
       ↓
  /joint_commands          (trajectory_msgs/JointTrajectory)

Prerequisites:
  • Conda env `vla` activated
  • ROS2 Humble sourced
  • Workspace built & sourced
  • ROS2 bridge node running (ros2 run mujoco_bridge bridge_node)

Run:
  source /opt/ros/humble/setup.bash
  source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
  PYTHONPATH=~/Desktop/VLA_Tutorial/VLATraining/sim:$PYTHONPATH \\
      /home/michael/miniforge3/envs/vla/bin/python3 \\
      ~/Desktop/VLA_Tutorial/VLATraining/sim/tools/real_vla_loop.py
"""

import os
import sys
import time
import pathlib
import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from tools.octo_adapter import OctoAdapter

# ROS2 imports
try:
    import rclpy
    from rclpy.node             import Node
    from sensor_msgs.msg        import Image
    from std_msgs.msg           import String
    from trajectory_msgs.msg    import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
except ImportError as e:
    raise ImportError(
        "ROS2 not found. Source ROS2 Humble and the mujoco_bridge workspace "
        "before running this script."
    ) from e

# JAX & Octo imports
try:
    import jax
    import cv2
    from octo.model.octo_model import OctoModel
except ImportError as e:
    raise ImportError(
        "Required ML dependencies (JAX, OpenCV, Octo) not found in this python environment. "
        "Run this script inside the 'vla' conda environment."
    ) from e


# ── Configuration ─────────────────────────────────────────────────────────────
PUBLISH_HZ     = 6.0      # target joint_commands publish rate
BRIDGE_TIMEOUT = 5.0      # seconds to wait for /bridge/status at startup
INSTRUCTION    = "pick up the metal box"
JOINT_LABELS   = ["A1", "A2", "A3", "A4", "A5", "A6"]


# ══════════════════════════════════════════════════════════════════════════════
#  RealVLANode
# ══════════════════════════════════════════════════════════════════════════════

class RealVLANode(Node):
    """
    Subscribes to /camera/wrist/image_raw.
    Publishes to /joint_commands at 6 Hz.
    Queries the locally loaded Octo-small-1.5 model on GPU for inference.
    """

    def __init__(self):
        super().__init__("real_vla_loop")

        self._adapter = OctoAdapter()
        self._rng     = jax.random.PRNGKey(0)

        # ── 1. Load Octo Model ────────────────────────────────────────────────
        self.get_logger().info("Loading Octo model ('rail-berkeley/octo-small-1.5') ...")
        self._model = OctoModel.load_pretrained("hf://rail-berkeley/octo-small-1.5")
        self.get_logger().info(f"Model successfully loaded on: {jax.devices()}")

        # ── 2. Create the task representation ─────────────────────────────────
        self.get_logger().info(f"Creating task with instruction: '{INSTRUCTION}'")
        self._task = self._model.create_tasks(texts=[INSTRUCTION])

        # ── 3. Warm-up JAX compilation ────────────────────────────────────────
        self.get_logger().info("Warming up JAX compilation (first-run JIT compiler) ...")
        t_warm_start = time.monotonic()
        fake_image = np.zeros((1, 1, 256, 256, 3), dtype=np.uint8)
        dummy_observation = {
            "image_primary": fake_image,
            "pad_mask": np.array([[True]]),
            "timestep_pad_mask": np.array([[True]])
        }
        # Run warm-up step to trigger JIT compilation
        _ = self._model.sample_actions(dummy_observation, self._task, rng=self._rng)
        self.get_logger().info(f"JAX warm-up complete! Compiled in {time.monotonic() - t_warm_start:.2f} s")

        # State
        self._last_image: np.ndarray | None = None
        self._bridge_alive = False
        self._publish_count = 0
        self._errors        = 0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, "/camera/wrist/image_raw",
            self._on_image, 10)

        self.create_subscription(
            String, "/bridge/status",
            self._on_bridge_status, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            JointTrajectory, "/joint_commands", 10)

        # ── 6 Hz timer for closed-loop control ───────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._inference_step)

        self.get_logger().info("[real_vla] Real VLA Loop Node Initialized and Ready")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_bridge_status(self, _msg: String) -> None:
        if not self._bridge_alive:
            self._bridge_alive = True
            self.get_logger().info("[real_vla] Bridge connection detected ✅")

    def _on_image(self, msg: Image) -> None:
        """Convert ROS2 Image to RGB numpy and store."""
        try:
            # ROS2 Image encoding is rgb8. Decode and reshape.
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            h, w = msg.height, msg.width
            self._last_image = raw.reshape((h, w, 3)).copy() # Keep as RGB
        except Exception as exc:
            self.get_logger().error(f"[real_vla] Image decoding failed: {exc}")
            self._errors += 1

    def _inference_step(self) -> None:
        """Run one 6-Hz real VLA inference step."""
        if self._last_image is None:
            # Bridge has not sent any images yet; send zeros to keep bridge healthy
            self._publish_zero()
            return

        try:
            t0 = time.monotonic()

            # 1. Resize the current camera frame to 256x256 using bilinear interpolation
            resized = cv2.resize(self._last_image, (256, 256), interpolation=cv2.INTER_LINEAR)

            # 2. Add batch and temporal history dimensions -> (1, 1, 256, 256, 3) uint8
            image_primary = np.expand_dims(np.expand_dims(resized, axis=0), axis=0)

            # 3. Assemble the observation dictionary
            observation = {
                "image_primary": image_primary,
                "pad_mask": np.array([[True]]),
                "timestep_pad_mask": np.array([[True]])
            }

            # 4. Perform GPU inference to sample a 7-DoF action sequence
            # rng key is updated slightly on each iteration to prevent JAX from locking
            self._rng, key = jax.random.split(self._rng)
            actions = self._model.sample_actions(observation, self._task, rng=key)

            # Octo model output shape is (batch=1, horizon=4, action_dim=7)
            # Take the immediate action step: actions[0, 0] -> (7,)
            action = np.asarray(actions[0, 0], dtype=np.float64)

            # Apply safety scaling to prevent crazy, unnormalized, massive oscillations.
            # Octo outputs normalized actions, but OctoAdapter treats them as raw radians.
            # Scaling down by 0.05 limits joint delta steps to a safe ~3 degrees max per step.
            action[:6] = action[:6] * 0.05

            # Convert normalized delta action to absolute joint positions & EM command
            cmd_dict = self._adapter.actions_to_joint_cmd(action)

            # 5. Publish absolute joint position trajectory command to ROS2
            self._publish_cmd(cmd_dict["joint_positions"])
            self._publish_count += 1

            latency = time.monotonic() - t0

            if self._publish_count % 18 == 0:  # Log every 3 seconds
                jp = cmd_dict["joint_positions"]
                self.get_logger().info(
                    f"[real_vla] step={self._publish_count:<4d} | "
                    f"latency={latency*1000.1:.1f}ms | "
                    f"A1={jp[0]:+.3f} A2={jp[1]:+.3f} A3={jp[2]:+.3f} | "
                    f"EM={cmd_dict['em_command']}")

        except Exception as exc:
            self.get_logger().error(f"[real_vla] VLA inference step failed: {exc}")
            self._errors += 1

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_cmd(self, positions) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_LABELS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(1e9 / PUBLISH_HZ))
        msg.points.append(pt)
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd([0.0] * 6)


# ══════════════════════════════════════════════════════════════════════════════
#  Bridge alive check
# ══════════════════════════════════════════════════════════════════════════════

def _check_bridge_alive(node: RealVLANode, timeout_s: float) -> bool:
    """Spin for up to timeout_s seconds, return True if bridge published status."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node._bridge_alive:
            return True
    return False


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    rclpy.init()
    node = RealVLANode()

    print(f"[real_vla] Probing bridge node (timeout = {BRIDGE_TIMEOUT} s) …")
    bridge_ok = _check_bridge_alive(node, BRIDGE_TIMEOUT)
    if not bridge_ok:
        node.get_logger().error(
            "[real_vla] ❌ Bridge not active. Start the ROS2 bridge first:\n"
            "  ros2 run mujoco_bridge bridge_node"
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ── Spin thread ───────────────────────────────────────────────────────────
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("[real_vla] Real VLA Loop successfully started! Close the viewer window to quit.")

    try:
        # Spin main thread infinitely
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[real_vla] Terminated by user.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
