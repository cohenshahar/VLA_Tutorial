"""
sim/task_tree/pick_and_place_tree.py  —  Phase 10, Task 10.12

Builds and returns the 7-node task tree for the metal_box pick-and-place task.
All postconditions evaluate purely against SensorReading fields.
"""

import sys
import pathlib
import numpy as np

# Path bootstrap to allow relative/absolute imports
_SIM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_SIM_DIR))

from task_tree.task_node import TaskNode
from task_tree.sensor_reading import SensorReading

# Scene constants
TABLE_Z = 0.85
TARGET_XY = np.array([0.5, 0.4])


def build_pick_and_place_tree() -> list[TaskNode]:
    """
    Constructs and returns the 7 subtask nodes in order.
    """
    nodes = [
        TaskNode(
            name="SEARCH",
            instruction="locate the metal box on the table",
            primitives=["observe_overhead", "observe_wrist"],
            postcondition=lambda r: np.all(np.isfinite(r.box_pos)),
            timeout_steps=500,
        ),
        TaskNode(
            name="APPROACH",
            instruction="move above the metal box",
            primitives=["move_to_preapproach", "descend_to_box"],
            postcondition=lambda r: 0.0 < r.proximity < 0.025,
            timeout_steps=5000,
        ),
        TaskNode(
            name="GRASP",
            instruction="grasp the metal box",
            primitives=["activate_em"],
            postcondition=lambda r: r.em_active and np.linalg.norm(r.ee_force) > 4.0,
            timeout_steps=2000,
        ),
        TaskNode(
            name="LIFT",
            instruction="lift the metal box up",
            primitives=["lift_to_carry_height"],
            postcondition=lambda r: (
                np.linalg.norm(r.ee_force) > 4.0 and
                r.box_pos[2] > TABLE_Z + 0.15
            ),
            timeout_steps=3000,
        ),
        TaskNode(
            name="LOCATE_TARGET",
            instruction="find the red target zone",
            primitives=["observe_overhead"],
            postcondition=lambda r: np.all(np.isfinite(r.ee_pos)),
            timeout_steps=500,
        ),
        TaskNode(
            name="TRANSPORT",
            instruction="move the box to the red target zone",
            primitives=["move_to_above_target"],
            postcondition=lambda r: (
                np.linalg.norm(r.ee_pos[:2] - TARGET_XY) < 0.08 and
                r.box_pos[2] > TABLE_Z + 0.08
            ),
            timeout_steps=6000,
        ),
        TaskNode(
            name="PLACE",
            instruction="release the metal box onto the target zone",
            primitives=["deactivate_em"],
            postcondition=lambda r: r.target_contact > 0.0 and not r.em_active,
            timeout_steps=2000,
        ),
    ]
    return nodes


if __name__ == "__main__":
    # Standalone test
    print("Running pick_and_place_tree standalone test...")
    nodes = build_pick_and_place_tree()
    
    assert len(nodes) == 7
    print(f"\nGenerated task tree with {len(nodes)} nodes:")
    for i, node in enumerate(nodes):
        print(f"  [{i+1}] {node.name:<15s} -> '{node.instruction}'")
        
    print("\n✅ Standalone tree verification PASSED!")
