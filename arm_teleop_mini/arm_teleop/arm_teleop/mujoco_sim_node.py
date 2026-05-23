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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

# import mujoco       # TODO Phase 2: uncomment when MuJoCo is installed
# import numpy as np  # TODO Phase 3


SIM_RATE_HZ = 50
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class MujocoSimNode(Node):
    """ROS2 node wrapping MuJoCo physics + state publishing."""

    def __init__(self):
        super().__init__("mujoco_sim_node")
        # TODO Phase 3:
        #   - declare ROS param `model_path` (default = models/kr6.xml)
        #   - declare ROS param `joint_limits_yaml` (default = config/kr6_params.yaml)
        #   - load MuJoCo model + data: self.model = mujoco.MjModel.from_xml_path(...)
        #   - load joint limits from YAML into self.q_min, self.q_max (np arrays of shape (6,))
        #   - create publisher /joint_states, /ee_pose
        #   - create subscription /cmd_joint_vel
        #   - create timer at 1/SIM_RATE_HZ -> self._step_simulation
        #   - keep last cmd + timestamp for deadman handling (Phase 5 cooperates here)
        pass

    def _step_simulation(self):
        """Timer callback. Step physics one tick, then publish state."""
        # TODO Phase 3:
        #   - mujoco.mj_step(self.model, self.data)
        #   - self._publish_joint_states()
        # TODO Phase 7:
        #   - self._publish_ee_pose()
        pass

    def _publish_joint_states(self):
        """Publish current q + qdot as sensor_msgs/JointState."""
        # TODO Phase 3:
        #   msg = JointState()
        #   msg.header.stamp = self.get_clock().now().to_msg()
        #   msg.name = JOINT_NAMES
        #   msg.position = list(self.data.qpos[:6])
        #   msg.velocity = list(self.data.qvel[:6])
        #   self.joint_state_pub.publish(msg)
        pass

    def _publish_ee_pose(self):
        """Publish end-effector pose in base frame."""
        # TODO Phase 7:
        #   - body_id = mujoco.mj_name2id(self.model, mjOBJ_BODY, "tool0")
        #   - xpos = self.data.xpos[body_id]
        #   - xquat = self.data.xquat[body_id]  (w, x, y, z order in MuJoCo!)
        #   - fill geometry_msgs/PoseStamped and publish on /ee_pose
        pass

    def _cmd_joint_vel_callback(self, msg: Float64MultiArray):
        """Receive desired joint velocities and apply (after clamping)."""
        # TODO Phase 4:
        #   - read msg.data into np.array of shape (6,)
        #   - store as self.cmd_qdot + self.last_cmd_time = now
        # TODO Phase 6:
        #   - call self._clamp_to_limits(...) before writing to actuators
        #   - write to self.data.ctrl[:6] (velocity actuators)
        pass

    def _clamp_to_limits(self, q, qdot):
        """Clamp commanded qdot so q stays inside [q_min, q_max]."""
        # TODO Phase 6:
        #   - for each joint i:
        #       if q[i] <= q_min[i] and qdot[i] < 0: qdot[i] = 0 and warn-once
        #       if q[i] >= q_max[i] and qdot[i] > 0: qdot[i] = 0 and warn-once
        #   - return clamped qdot
        raise NotImplementedError("Phase 6")


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
