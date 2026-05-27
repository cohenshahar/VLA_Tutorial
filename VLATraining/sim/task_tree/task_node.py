"""
sim/task_tree/task_node.py  —  Phase 10, Task 10.11

Represents a single subtask node in the Task Tree.
Evaluates its postcondition, manages steps active, and tracks status.
"""

from dataclasses import dataclass, field
from typing import Callable, List, TYPE_CHECKING

if TYPE_CHECKING:
    from task_tree.sensor_reading import SensorReading


@dataclass
class TaskNode:
    name:           str
    instruction:    str                               # language instruction fed to VLA
    primitives:     List[str]                         # named sub-actions (for logging)
    postcondition:  Callable[["SensorReading"], bool] # returns True when subtask is done
    timeout_steps:  int  = 3000                       # fail if not done in this many steps
    status:         str  = "pending"                  # pending | active | done | failed
    steps_active:   int  = field(default=0, repr=False)

    def tick(self, reading: "SensorReading") -> str:
        """
        Call once per simulation step when this node is active.
        Increments steps_active, evaluates postcondition, updates status.
        Returns new status string.
        """
        if self.status not in ["active", "pending"]:
            return self.status

        self.status = "active"
        self.steps_active += 1

        # Check postcondition
        if self.postcondition(reading):
            self.status = "done"
            return "done"

        # Check timeout
        if self.steps_active >= self.timeout_steps:
            self.status = "failed"
            return "failed"

        return "active"


if __name__ == "__main__":
    # Standalone test
    print("Running TaskNode standalone test...")

    # Mock sensor reading class
    class MockSensorReading:
        def __init__(self, val):
            self.val = val

    # Test 1: Succeeds on postcondition
    node = TaskNode(
        name="TEST_SUCCESS",
        instruction="success test instruction",
        primitives=["prim1"],
        postcondition=lambda r: r.val > 5,
        timeout_steps=10
    )

    assert node.status == "pending"
    assert node.steps_active == 0

    # Step 1-3: condition false
    for i in range(1, 4):
        status = node.tick(MockSensorReading(2))
        assert status == "active"
        assert node.status == "active"
        assert node.steps_active == i

    # Step 4: condition becomes true
    status = node.tick(MockSensorReading(6))
    assert status == "done"
    assert node.status == "done"
    print("  ✅ Success path test PASSED!")

    # Test 2: Fails on timeout
    node_timeout = TaskNode(
        name="TEST_TIMEOUT",
        instruction="timeout test instruction",
        primitives=["prim1"],
        postcondition=lambda r: r.val > 5,
        timeout_steps=5
    )

    for i in range(1, 5):
        status = node_timeout.tick(MockSensorReading(2))
        assert status == "active"
        assert node_timeout.steps_active == i

    status = node_timeout.tick(MockSensorReading(2))
    assert status == "failed"
    assert node_timeout.status == "failed"
    assert node_timeout.steps_active == 5
    print("  ✅ Timeout path test PASSED!")

    print("\n✅ All TaskNode standalone tests PASSED!")
