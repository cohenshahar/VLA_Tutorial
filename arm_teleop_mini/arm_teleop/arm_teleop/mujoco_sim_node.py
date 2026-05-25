"""
mujoco_sim_node — the simulator + state publisher.

Introduced in Phase 3. Extended in Phases 4, 6, 7.

Responsibilities:
  - Load MuJoCo model from models/<ee>.xml (path from ROS param).
  - Step physics at 50 Hz via a ROS2 timer.
  - Publish /joint_states (sensor_msgs/JointState) at 50 Hz.
  - Publish /ee_pose (geometry_msgs/PoseStamped) at 50 Hz.
  - Subscribe to /cmd_joint_vel (std_msgs/Float64MultiArray, 6 floats).
  - Clamp commands to joint limits read from config/kr6_params.yaml.

Owns topics:
  - PUB  /joint_states           sensor_msgs/JointState        @ 50 Hz
  - PUB  /ee_pose                geometry_msgs/PoseStamped     @ 50 Hz
  - SUB  /cmd_joint_vel          std_msgs/Float64MultiArray
"""

import os

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


SIM_RATE_HZ = 50
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class MujocoSimNode(Node):
    """ROS2 node wrapping MuJoCo physics + state publishing."""

    def __init__(self):
        super().__init__("mujoco_sim_node")

        # ── ROS parameters ────────────────────────────────────────────────
        pkg_share = get_package_share_directory("arm_teleop")
        default_model = os.path.join(pkg_share, "models", "kr6.xml")
        default_yaml  = os.path.join(pkg_share, "config", "kr6_params.yaml")

        self.declare_parameter("model_path",        default_model)
        self.declare_parameter("joint_limits_yaml", default_yaml)

        model_path  = self.get_parameter("model_path").get_parameter_value().string_value
        limits_yaml = self.get_parameter("joint_limits_yaml").get_parameter_value().string_value

        # ── MuJoCo ────────────────────────────────────────────────────────
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data  = mujoco.MjData(self.model)
        self.get_logger().info(f"Loaded MuJoCo model: {model_path}")

        # ── Joint limits ──────────────────────────────────────────────────
        with open(limits_yaml, "r") as f:
            cfg = yaml.safe_load(f)
        limits = cfg["joint_limits"]
        self.q_min = np.array([limits[j]["min"] for j in JOINT_NAMES])
        self.q_max = np.array([limits[j]["max"] for j in JOINT_NAMES])
        self._limit_warned: set[str] = set()   # warn-once keys

        # ── Publishers ────────────────────────────────────────────────────
        self.joint_state_pub = self.create_publisher(JointState,    "/joint_states", 10)
        self.ee_pose_pub     = self.create_publisher(PoseStamped,   "/ee_pose",      10)

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            "/cmd_joint_vel",
            self._cmd_joint_vel_callback,
            10,
        )
        self.cmd_qdot      = np.zeros(6)
        self.last_cmd_time = self.get_clock().now()

        # ── Vacuum visual subscriber ──────────────────────────────────────
        self.create_subscription(Bool, "/vacuum/state", self._on_vacuum_state, 10)
        # Geom names that change color when vacuum is on
        self._vacuum_geoms   = ["cup_neck", "cup_bell_body", "cup_bell_lip"]
        self._vacuum_on_rgba  = [0.1, 0.8, 0.1, 1.0]   # green = active
        self._vacuum_off_rgba = [0.6, 0.15, 0.80, 1.0]  # purple = off
        self._dt = 1.0 / SIM_RATE_HZ

        # Initialise ctrl to current qpos so the arm holds its pose on start
        self.data.ctrl[:6] = self.data.qpos[:6].copy()

        # ── EE body ID (cached once) ───────────────────────────────────────
        self._ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link6")
        if self._ee_body_id == -1:
            self.get_logger().error("Body 'link6' not found in model — /ee_pose will not publish")

        # ── Passive viewer (live 3-D window) ─────────────────────────────
        self._viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # ── Timer ─────────────────────────────────────────────────────────
        self.create_timer(self._dt, self._step_simulation)
        self.get_logger().info(f"mujoco_sim_node running at {SIM_RATE_HZ} Hz")

    def _step_simulation(self):
        """Timer callback. Step physics one tick, then publish state."""
        # Integrate velocity command into position setpoint (clamped)
        safe_qdot = self._clamp_to_limits(self.data.qpos[:6], self.cmd_qdot.copy())
        self.data.ctrl[:6] += safe_qdot * self._dt
        # Hard-clip ctrl to joint range so numerical drift never escapes
        np.clip(self.data.ctrl[:6], self.q_min, self.q_max, out=self.data.ctrl[:6])
        mujoco.mj_step(self.model, self.data)
        self._publish_joint_states()
        self._publish_ee_pose()
        if self._viewer.is_running():
            self._viewer.sync()

    def _publish_joint_states(self):
        """Publish current q + qdot as sensor_msgs/JointState."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = JOINT_NAMES
        msg.position     = self.data.qpos[:6].tolist()
        msg.velocity     = self.data.qvel[:6].tolist()
        self.joint_state_pub.publish(msg)

    def _publish_ee_pose(self):
        """Publish end-effector pose (link6 frame) in world/base frame."""
        if self._ee_body_id == -1:
            return
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"
        # Position
        pos = self.data.xpos[self._ee_body_id]
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        # Orientation — MuJoCo: [w, x, y, z]  →  ROS: [x, y, z, w]
        quat = self.data.xquat[self._ee_body_id]
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        self.ee_pose_pub.publish(msg)

    def _cmd_joint_vel_callback(self, msg: Float64MultiArray):
        """Receive desired joint velocities and apply (after clamping)."""
        if len(msg.data) != 6:
            self.get_logger().warn(f"Expected 6 velocities, got {len(msg.data)} — ignoring")
            return
        self.cmd_qdot = np.array(msg.data, dtype=float)
        self.last_cmd_time = self.get_clock().now()

    def _on_vacuum_state(self, msg: Bool):
        """Swap suction cup geom colors to reflect vacuum on/off state."""
        rgba = self._vacuum_on_rgba if msg.data else self._vacuum_off_rgba
        for name in self._vacuum_geoms:
            geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom_id >= 0:
                self.model.geom_rgba[geom_id] = rgba

    def _clamp_to_limits(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """Clamp commanded qdot so q stays inside [q_min, q_max]. Warn once per joint."""
        for i, name in enumerate(JOINT_NAMES):
            if q[i] <= self.q_min[i] and qdot[i] < 0.0:
                key = f"{name}_min"
                if key not in self._limit_warned:
                    self.get_logger().warn(f"{name} at lower limit ({self.q_min[i]:.3f} rad) — clamping")
                    self._limit_warned.add(key)
                qdot[i] = 0.0
            elif q[i] >= self.q_max[i] and qdot[i] > 0.0:
                key = f"{name}_max"
                if key not in self._limit_warned:
                    self.get_logger().warn(f"{name} at upper limit ({self.q_max[i]:.3f} rad) — clamping")
                    self._limit_warned.add(key)
                qdot[i] = 0.0
            else:
                # Joint moved away from limit — allow warning again next time
                self._limit_warned.discard(f"{name}_min")
                self._limit_warned.discard(f"{name}_max")
        return qdot


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# *VLA Research | Shahar Cohen | BGU Mechatronics | 2026-05-24*
