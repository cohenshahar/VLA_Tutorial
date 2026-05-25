"""
cartesian_controller_node — Jacobian IK controller.

Introduced in Phase 9. Extended in Phase 10 (frame toggle + position tracking).

Subscribes to:
  - /joint_states  (sensor_msgs/JointState)          — current q
  - /ee_pose       (geometry_msgs/PoseStamped)       — current EE pose
  - /cmd_ee_vel    (geometry_msgs/TwistStamped)      — desired EE velocity
                                                       header.frame_id = "base" or "tool"

Publishes:
  - /cmd_joint_vel (std_msgs/Float64MultiArray)      — computed q_dot

Algorithm (Phase 9 + 10):
  1. Integrate incoming velocity into a target position (while command is fresh).
  2. Compute position error: e = target_pos - current_pos.
  3. x_dot = Kp * e  (position-only; keeps EE on a straight path).
  4. q_dot = DLS(J, x_dot)  — damped least squares IK.

Frame handling (Phase 10):
  - header.frame_id == "base": velocity used as-is.
  - header.frame_id == "tool": linear velocity rotated by R_base_tool from /ee_pose.
"""

import csv
import os
import time

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .kinematics import compute_jacobian, damped_least_squares_ik, quat_to_rotation_matrix

IK_LAMBDA_SQ   = 0.01
KP_POSITION    = 0.5     # proportional gain: error [m] → x_dot [m/s]
MAX_EE_VEL     = 0.15    # m/s  — clamp commanded EE speed
MAX_QDOT       = 2.0     # rad/s — clamp each joint velocity
CTRL_RATE_HZ   = 50
DEADMAN_S      = 0.3     # stop integrating target if no cmd for this long


class CartesianControllerNode(Node):
    def __init__(self):
        super().__init__("cartesian_controller_node")

        # ── MuJoCo model (read-only, for Jacobian only) ───────────────────
        pkg_share = get_package_share_directory("arm_teleop")
        default_model = os.path.join(pkg_share, "models", "kr6.xml")
        self.declare_parameter("model_path", default_model)
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data  = mujoco.MjData(self.model)
        self.get_logger().info(f"Cartesian controller loaded model: {model_path}")

        # ── State ─────────────────────────────────────────────────────────
        self.q           = np.zeros(6)
        self.current_pos = None          # (3,) from /ee_pose, None until first msg
        self.target_pos  = None          # (3,) integrated target position
        self.R_base_tool = np.eye(3)     # rotation matrix: tool → base
        self.last_vel    = np.zeros(3)   # latest base-frame linear velocity command
        self.last_cmd_t  = 0.0

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(JointState,   "/joint_states", self._on_joint_states, 10)
        self.create_subscription(PoseStamped,  "/ee_pose",      self._on_ee_pose,      10)
        self.create_subscription(TwistStamped, "/cmd_ee_vel",   self._on_cmd_ee_vel,   10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, "/cmd_joint_vel", 10)

        # ── 50 Hz control timer ───────────────────────────────────────────
        self.create_timer(1.0 / CTRL_RATE_HZ, self._control_tick)

        # ── CSV logger ────────────────────────────────────────────────────
        log_path = "/tmp/cartesian_ctrl_log.csv"
        self._csv_file = open(log_path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow([
            "t", "cond",
            "cur_x","cur_y","cur_z",
            "tgt_x","tgt_y","tgt_z",
            "err_x","err_y","err_z",
            "xdot_x","xdot_y","xdot_z",
            "qdot_1","qdot_2","qdot_3","qdot_4","qdot_5","qdot_6",
        ])
        self._t0 = time.monotonic()
        self.get_logger().info(f"CSV log → {log_path}")

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()

    def _on_joint_states(self, msg: JointState):
        self.q = np.array(msg.position[:6])

    def _on_ee_pose(self, msg: PoseStamped):
        p = msg.pose.position
        pos = np.array([p.x, p.y, p.z])
        self.current_pos = pos
        if self.target_pos is None:
            self.target_pos = pos.copy()   # initialise target on first message
        o = msg.pose.orientation
        quat = np.array([o.w, o.x, o.y, o.z])
        self.R_base_tool = quat_to_rotation_matrix(quat)

    def _on_cmd_ee_vel(self, msg: TwistStamped):
        v = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        if msg.header.frame_id == "tool":
            v = self.R_base_tool @ v        # rotate tool → base
        self.last_vel   = v
        self.last_cmd_t = time.monotonic()

    def _control_tick(self):
        if self.current_pos is None or self.target_pos is None:
            return

        dt = 1.0 / CTRL_RATE_HZ

        # Integrate target while command is fresh
        if time.monotonic() - self.last_cmd_t < DEADMAN_S:
            self.target_pos = self.target_pos + self.last_vel * dt
            # Leash: don't let target run more than 3 cm ahead of current pos
            diff = self.target_pos - self.current_pos
            dist = np.linalg.norm(diff)
            MAX_LEASH = 0.03
            if dist > MAX_LEASH:
                self.target_pos = self.current_pos + diff * (MAX_LEASH / dist)

        # Position error → desired EE velocity (position axes only)
        pos_error = self.target_pos - self.current_pos
        x_dot = np.zeros(6)
        x_dot[:3] = KP_POSITION * pos_error

        # Clamp linear speed so arm can't slam at high velocity
        speed = np.linalg.norm(x_dot[:3])
        if speed > MAX_EE_VEL:
            x_dot[:3] *= MAX_EE_VEL / speed

        # Jacobian + DLS
        self.data.qpos[:6] = self.q
        mujoco.mj_forward(self.model, self.data)
        J = compute_jacobian(self.model, self.data, body_name="link6")

        cond = np.linalg.cond(J)
        if cond > 5000.0:
            self.get_logger().warn(f"IK_FAIL: singularity (cond={cond:.0f})", throttle_duration_sec=2.0)
            q_dot = np.zeros(6)
        else:
            q_dot = damped_least_squares_ik(J, x_dot, IK_LAMBDA_SQ)
            q_dot = np.clip(q_dot, -MAX_QDOT, MAX_QDOT)

        out = Float64MultiArray()
        out.data = q_dot.tolist()
        self.joint_vel_pub.publish(out)

        # ── Log to CSV ────────────────────────────────────────────────────
        t = time.monotonic() - self._t0
        cp = self.current_pos
        tp = self.target_pos
        e  = tp - cp
        self._csv.writerow([
            f"{t:.4f}", f"{cond:.1f}",
            f"{cp[0]:.5f}", f"{cp[1]:.5f}", f"{cp[2]:.5f}",
            f"{tp[0]:.5f}", f"{tp[1]:.5f}", f"{tp[2]:.5f}",
            f"{e[0]:.5f}",  f"{e[1]:.5f}",  f"{e[2]:.5f}",
            f"{x_dot[0]:.5f}", f"{x_dot[1]:.5f}", f"{x_dot[2]:.5f}",
        ] + [f"{v:.5f}" for v in q_dot])


def main(args=None):
    rclpy.init(args=args)
    node = CartesianControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
