"""
sim/task_tree/task_tree_manager.py  —  Phase 10, Task 10.13

Manages the execution flow of the Task Tree.
Evaluates the active node at each step, handles transitions, and exposes status diagnostics.
"""

import sys
import pathlib
import numpy as np

# Path bootstrap to allow relative/absolute imports
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from task_tree.task_node import TaskNode
from task_tree.sensor_reading import SensorReading


class TaskTreeManager:
    """
    State manager and transition arbiter for the Task Tree.
    """
    def __init__(self, nodes: list[TaskNode]):
        self._nodes = nodes
        self._idx   = 0
        self._activate_current()

    def _activate_current(self):
        if self._idx < len(self._nodes):
            self._nodes[self._idx].status = "active"

    def step(self, reading: SensorReading) -> str:
        """
        Call once per simulation step.
        Ticks the active node, advances index on subtask completion.
        Returns the language instruction for the current active node.
        """
        if self.is_complete() or self.is_failed():
            return self.get_current_instruction()

        active_node = self._nodes[self._idx]
        status = active_node.tick(reading)

        if status == "done":
            self._idx += 1
            self._activate_current()

        return self.get_current_instruction()

    def get_current_instruction(self) -> str:
        """Return the VLA instruction for the current subtask."""
        if self.is_complete():
            return "task complete"
        if self.is_failed():
            return "task failed"
        return self._nodes[self._idx].instruction

    def get_status_dict(self) -> dict:
        """
        Return a JSON-serializable status dictionary suitable for ROS2 status publishing.
        """
        if self.is_complete():
            active_name = "COMPLETE"
            active_instr = "task complete"
            steps = 0
        elif self.is_failed():
            active_name = "FAILED"
            active_instr = "task failed"
            steps = 0
        else:
            active_node = self._nodes[self._idx]
            active_name = active_node.name
            active_instr = active_node.instruction
            steps = active_node.steps_active

        return {
            "active_node":   active_name,
            "active_idx":    self._idx,
            "instruction":   active_instr,
            "steps_active":  steps,
            "node_statuses": [{"name": n.name, "status": n.status} for n in self._nodes]
        }

    def is_complete(self) -> bool:
        """True when the last node reaches 'done'."""
        return self._idx >= len(self._nodes)

    def is_failed(self) -> bool:
        """True when any node has failed."""
        return any(n.status == "failed" for n in self._nodes)

    def reset(self):
        """Reset all nodes to pending, restart from node 0."""
        self._idx = 0
        for node in self._nodes:
            node.status = "pending"
            node.steps_active = 0
        self._activate_current()


if __name__ == "__main__":
    # Standalone test simulating a complete pick-and-place run using dummy SensorReadings
    print("Running TaskTreeManager standalone test...")
    
    from task_tree.pick_and_place_tree import build_pick_and_place_tree
    
    nodes = build_pick_and_place_tree()
    manager = TaskTreeManager(nodes)
    
    # ── Create a base dummy sensor reading ─────────────────────────────────────
    class DummyReading:
        def __init__(self):
            self.t = 0.0
            self.qpos = np.zeros(6)
            self.qvel = np.zeros(6)
            self.torque = np.zeros(6)
            self.ee_force = np.zeros(3)
            self.ee_torque = np.zeros(3)
            self.proximity = -1.0
            self.em_active = False
            self.target_contact = 0.0
            self.overhead_bgr = None
            self.wrist_bgr = None
            self.box_pos = np.array([0.76, 0.0, 0.90])  # starts finite (SEARCH passes)
            self.ee_pos = np.array([0.99, -0.3, 1.285])  # starts finite (LOCATE_TARGET passes)

    r = DummyReading()

    print("\nStarting mock Task Tree traversal:")
    
    # 1. Step SEARCH: box_pos is already finite
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  SEARCH -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "APPROACH"

    # 2. Step APPROACH: mock rangefinder reading
    r.proximity = 0.015
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  APPROACH -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "GRASP"

    # 3. Step GRASP: mock weld active and force
    r.em_active = True
    r.ee_force = np.array([0, 0, 5.0])
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  GRASP -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "LIFT"

    # 4. Step LIFT: mock lift height
    r.box_pos[2] = 1.15
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  LIFT -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "LOCATE_TARGET"

    # 5. Step LOCATE_TARGET: ee_pos is already finite
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  LOCATE_TARGET -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "TRANSPORT"

    # 6. Step TRANSPORT: mock proximity to target [0.5, 0.4]
    r.ee_pos = np.array([0.51, 0.41, 1.15])
    r.box_pos[2] = 1.05
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  TRANSPORT -> done. Next subtask: {status['active_node']} -> '{instr}'")
    assert status["active_node"] == "PLACE"

    # 7. Step PLACE: mock EM deactivation and target contact
    r.em_active = False
    r.target_contact = 50.0
    instr = manager.step(r)
    status = manager.get_status_dict()
    print(f"  PLACE -> done. Status: {status['active_node']} -> '{instr}'")
    
    assert manager.is_complete()
    assert not manager.is_failed()
    
    # Test reset
    manager.reset()
    assert manager._idx == 0
    assert not manager.is_complete()
    assert manager.get_status_dict()["active_node"] == "SEARCH"
    print("  ✅ Tree reset PASSED!")

    print("\n✅ TaskTreeManager standalone test successfully completed!")
