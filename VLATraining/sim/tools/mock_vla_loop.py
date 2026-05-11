"""
tools/mock_vla_loop.py  —  Phase 10, Task 10.9
═══════════════════════════════════════════════════════════════════════════════
Closed-loop mock VLA inference loop — no real model, no GPU required.

Pipeline (6 Hz):
  /camera/wrist/image_raw  (ROS2 sensor_msgs/Image)
       ↓
  ObsPipeline.get_observation()   →  {"image": (256,256,3) float32, "instruction": str}
       ↓
  random 7-float action            (mocks Octo inference)
       ↓
  OctoAdapter.actions_to_joint_cmd()  →  {"joint_positions": [...], "em_command": bool}
       ↓
  /joint_commands  (trajectory_msgs/JointTrajectory, 6 Hz)

Done when: loop runs for 30 s without crash.
Bridge alive check: startup probe of /bridge/status — prints error and exits
if the bridge is not publishing within 5 s.

Prerequisites
  • Bridge node running (ros2 run mujoco_bridge bridge_node)
  • ROS2 Humble sourced
  • VLA workspace sourced (install/setup.bash)

Run:
  source /opt/ros/humble/setup.bash
  source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
  PYTHONPATH=~/Desktop/VLA_Tutorial/VLATraining/sim \\
      python3 ~/Desktop/VLA_Tutorial/VLATraining/sim/tools/mock_vla_loop.py

Verify rate:
  ros2 topic hz /joint_commands          # expect ~6 Hz
"""

import pathlib
import sys
import time

import numpy as np

# ── Path bootstrap ────────────────────────────────────────────────────────────
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from tools.obs_pipeline  import ObsPipeline
from tools.octo_adapter  import OctoAdapter

# ROS2 imports (must have ROS2 sourced)
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

# ── Configuration ─────────────────────────────────────────────────────────────
RUN_DURATION_S  = 30.0    # total mock-loop duration (seconds)
PUBLISH_HZ      = 6.0     # target joint_commands publish rate
BRIDGE_TIMEOUT  = 5.0     # seconds to wait for /bridge/status at startup

INSTRUCTION     = "pick up the metal box"
JOINT_LABELS    = ["A1", "A2", "A3", "A4", "A5", "A6"]


# ══════════════════════════════════════════════════════════════════════════════
#  MockVLANode
# ══════════════════════════════════════════════════════════════════════════════

class MockVLANode(Node):
    """
    Subscribes to /camera/wrist/image_raw.
    Publishes to /joint_commands at PUBLISH_HZ.
    Generates random 7-float actions (mock Octo inference).
    """

    def __init__(self):
        super().__init__("mock_vla_loop")

        self._pipeline = ObsPipeline()
        self._adapter  = OctoAdapter()
        self._rng      = np.random.default_rng()

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

        # ── 6 Hz inference + publish timer ────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._inference_step)

        self.get_logger().info(
            f"[mock_vla] Node started — will publish at {PUBLISH_HZ} Hz "
            f"for {RUN_DURATION_S:.0f} s")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_bridge_status(self, _msg: String) -> None:
        if not self._bridge_alive:
            self._bridge_alive = True
            self.get_logger().info("[mock_vla] Bridge alive ✅")

    def _on_image(self, msg: Image) -> None:
        """Convert ROS2 Image to BGR numpy and store."""
        try:
            # Encoding is rgb8 (bridge publishes RGB); convert to BGR for ObsPipeline
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            h, w = msg.height, msg.width
            rgb = raw.reshape((h, w, 3))
            self._last_image = rgb[:, :, ::-1].copy()   # RGB → BGR
        except Exception as exc:
            self.get_logger().error(f"[mock_vla] Image decode error: {exc}")
            self._errors += 1

    def _inference_step(self) -> None:
        """Run one 6-Hz pipeline iteration."""
        if self._last_image is None:
            # No image yet — send zero command to keep bridge happy
            self._publish_zero()
            return

        try:
            # 1. Observation
            obs = self._pipeline.get_observation(
                self._last_image, INSTRUCTION)

            # 2. Mock action (random 7 floats in [-0.05, +0.05])
            actions = self._rng.uniform(-0.05, 0.05, size=7).astype(np.float32)
            actions[6] = 0.0   # keep EM off during mock loop

            # 3. Adapter → command dict
            cmd_dict = self._adapter.actions_to_joint_cmd(actions)

            # 4. Publish
            self._publish_cmd(cmd_dict["joint_positions"])
            self._publish_count += 1

            if self._publish_count % 18 == 0:   # log every 3 s
                jp = cmd_dict["joint_positions"]
                self.get_logger().info(
                    f"[mock_vla] step {self._publish_count:4d} | "
                    f"J1={jp[0]:+.3f} J2={jp[1]:+.3f} J3={jp[2]:+.3f} | "
                    f"img {obs['image'].shape}")

        except Exception as exc:
            self.get_logger().error(f"[mock_vla] inference error: {exc}")
            self._errors += 1

    # ── Publish helpers ───────────────────────────────────────────────────────

    def _publish_cmd(self, positions) -> None:
        msg = JointTrajectory()
        msg.joint_names = JOINT_LABELS
        pt = JointTrajectoryPoint()
        pt.positions    = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=0, nanosec=int(1e9 / PUBLISH_HZ))
        msg.points.append(pt)
        self._cmd_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._publish_cmd([0.0] * 6)


# ══════════════════════════════════════════════════════════════════════════════
#  Bridge alive check
# ══════════════════════════════════════════════════════════════════════════════

def _check_bridge_alive(node: MockVLANode, timeout_s: float) -> bool:
    """
    Spin for up to timeout_s seconds, return True if bridge published status.
    """
    import threading
    result = [False]
    spin_thread = threading.Thread(
        target=rclpy.spin_once, args=(node,), kwargs={"timeout_sec": 1.0},
        daemon=True)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node._bridge_alive:
            result[0] = True
            break
    return result[0]


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Run the mock VLA loop for RUN_DURATION_S seconds."""
    rclpy.init()
    node = MockVLANode()

    print(f"[mock_vla] Checking bridge status (timeout = {BRIDGE_TIMEOUT} s) …")
    bridge_ok = _check_bridge_alive(node, BRIDGE_TIMEOUT)
    if not bridge_ok:
        node.get_logger().error(
            "[mock_vla] ❌ Bridge not alive after "
            f"{BRIDGE_TIMEOUT} s. "
            "Start the bridge first:\n"
            "  ros2 run mujoco_bridge bridge_node")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ── Main loop (spin + duration check) ─────────────────────────────────────
    import threading
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < RUN_DURATION_S:
            elapsed = time.monotonic() - t_start
            print(f"\r[mock_vla] {elapsed:5.1f}/{RUN_DURATION_S:.0f} s  "
                  f"published={node._publish_count:4d}  errors={node._errors}  ",
                  end="", flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[mock_vla] Interrupted by user.")

    elapsed = time.monotonic() - t_start
    print()
    print(f"\n[mock_vla] Run complete.")
    print(f"  Duration   : {elapsed:.1f} s")
    print(f"  Published  : {node._publish_count} commands")
    print(f"  Actual Hz  : {node._publish_count / elapsed:.2f} Hz  "
          f"(target {PUBLISH_HZ:.0f} Hz)")
    print(f"  Errors     : {node._errors}")

    if node._publish_count > 0 and node._errors == 0:
        print("  ✅ DONE — loop ran without crash.")
    else:
        print("  ❌ Loop had errors or no publishes. Check logs above.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
