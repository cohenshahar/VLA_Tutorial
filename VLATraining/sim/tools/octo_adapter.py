"""
tools/octo_adapter.py  —  Phase 10, Task 10.8
═══════════════════════════════════════════════════════════════════════════════
OctoAdapter: convert a raw Octo action array into joint commands and EM state.

Octo output format
  7 floats: [Δjoint_1, Δjoint_2, Δjoint_3, Δjoint_4, Δjoint_5, Δjoint_6,
              gripper]
  • Δjoint_i  — joint position delta (radians, to be added to current pos)
  • gripper   — continuous [0, 1]; > 0.5 → EM activate command

Joint limits (from arm.xml): all joints ±2.8 rad.

Output dict
  "joint_positions" : list[float]  — 6 absolute joint positions (rad), clipped
  "em_command"      : bool         — True → activate EM; False → deactivate

Standalone test:
  python3 VLATraining/sim/tools/octo_adapter.py
"""

import numpy as np

# ── Joint limits ──────────────────────────────────────────────────────────────
_JOINT_LIMIT  = 2.8        # ±2.8 rad for all 6 joints
_GRIPPER_THRESH = 0.5      # gripper > this → EM ON


class OctoAdapter:
    """
    Stateful adapter that converts Octo's 7-float action into robot commands.

    The adapter is stateful: it accumulates joint deltas onto the last known
    joint positions.  Call reset() or set_current_pos() when the robot
    position is known (e.g. at the start of a session, or from /joint_states).

    Usage
    -----
    adapter = OctoAdapter()
    adapter.set_current_pos([0.0] * 6)            # seed from /joint_states
    cmd = adapter.actions_to_joint_cmd(actions)   # actions: (7,) ndarray
    # cmd["joint_positions"] → list[float] (6, clipped to ±2.8 rad)
    # cmd["em_command"]      → bool
    """

    def __init__(self):
        self._qpos = np.zeros(6, dtype=np.float64)  # current joint positions

    # ── Public API ────────────────────────────────────────────────────────────

    def set_current_pos(self, positions) -> None:
        """
        Seed the adapter with the current robot joint positions.

        Parameters
        ----------
        positions : array-like, length 6 — joint positions in radians.
        """
        self._qpos = np.array(positions, dtype=np.float64)
        if self._qpos.shape != (6,):
            raise ValueError(
                f"Expected 6 joint positions; got shape {self._qpos.shape}")

    def reset(self) -> None:
        """Reset joint positions to all-zero (home pose)."""
        self._qpos = np.zeros(6, dtype=np.float64)

    def actions_to_joint_cmd(self, actions: np.ndarray) -> dict:
        """
        Convert Octo's 7-float action into a robot command dict.

        Parameters
        ----------
        actions : np.ndarray, shape (7,)
            [Δj1, Δj2, Δj3, Δj4, Δj5, Δj6, gripper]

        Returns
        -------
        dict with:
            "joint_positions" : list[float]  — 6 absolute positions (rad),
                                               clipped to [−2.8, +2.8]
            "em_command"      : bool         — True = EM ON, False = EM OFF
        """
        actions = np.asarray(actions, dtype=np.float64)
        if actions.shape != (7,):
            raise ValueError(
                f"actions must have shape (7,); got {actions.shape}")

        deltas  = actions[:6]
        gripper = float(actions[6])

        # Accumulate deltas, then clip to joint limits
        self._qpos = np.clip(self._qpos + deltas,
                             -_JOINT_LIMIT, _JOINT_LIMIT)

        return {
            "joint_positions": self._qpos.tolist(),
            "em_command":      gripper > _GRIPPER_THRESH,
        }


# ══════════════════════════════════════════════════════════════════════════════
#  Standalone test
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Smoke-test OctoAdapter with known input/output pairs."""
    adapter = OctoAdapter()

    print("── OctoAdapter smoke test ──────────────────────────────────")

    # Test 1 — zero deltas, gripper open
    adapter.reset()
    cmd = adapter.actions_to_joint_cmd(np.zeros(7))
    assert cmd["joint_positions"] == [0.0] * 6, \
        f"Expected [0]*6, got {cmd['joint_positions']}"
    assert cmd["em_command"] is False, \
        f"Expected em_command=False, got {cmd['em_command']}"
    print("  Test 1 (zero action, open): ✅")

    # Test 2 — gripper close (actions[6] = 0.8 > 0.5)
    adapter.reset()
    a = np.zeros(7);  a[6] = 0.8
    cmd = adapter.actions_to_joint_cmd(a)
    assert cmd["em_command"] is True, "Expected em_command=True"
    print("  Test 2 (gripper close):     ✅")

    # Test 3 — joint clipping: large positive delta on joint 0
    adapter.reset()
    a = np.zeros(7);  a[0] = 5.0   # 5 rad > 2.8 → must clip
    cmd = adapter.actions_to_joint_cmd(a)
    assert abs(cmd["joint_positions"][0] - _JOINT_LIMIT) < 1e-9, \
        f"Expected {_JOINT_LIMIT}, got {cmd['joint_positions'][0]}"
    print("  Test 3 (upper clip):        ✅")

    # Test 4 — joint clipping: large negative delta
    adapter.reset()
    a = np.zeros(7);  a[3] = -5.0
    cmd = adapter.actions_to_joint_cmd(a)
    assert abs(cmd["joint_positions"][3] - (-_JOINT_LIMIT)) < 1e-9, \
        f"Expected {-_JOINT_LIMIT}, got {cmd['joint_positions'][3]}"
    print("  Test 4 (lower clip):        ✅")

    # Test 5 — delta accumulation across two steps
    adapter.reset()
    adapter.set_current_pos([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    a1 = np.zeros(7);  a1[0] = 0.3   # qpos[0] → 0.8
    cmd = adapter.actions_to_joint_cmd(a1)
    assert abs(cmd["joint_positions"][0] - 0.8) < 1e-9, \
        f"Expected 0.8, got {cmd['joint_positions'][0]}"
    a2 = np.zeros(7);  a2[0] = 0.2   # qpos[0] → 1.0
    cmd = adapter.actions_to_joint_cmd(a2)
    assert abs(cmd["joint_positions"][0] - 1.0) < 1e-9, \
        f"Expected 1.0, got {cmd['joint_positions'][0]}"
    print("  Test 5 (accumulation):      ✅")

    # Test 6 — gripper threshold boundary
    adapter.reset()
    a_lo = np.zeros(7);  a_lo[6] = 0.5    # exactly at threshold → OFF
    a_hi = np.zeros(7);  a_hi[6] = 0.501  # just above → ON
    assert adapter.actions_to_joint_cmd(a_lo)["em_command"] is False
    assert adapter.actions_to_joint_cmd(a_hi)["em_command"] is True
    print("  Test 6 (threshold):         ✅")

    print(f"\nJoint limit  : ±{_JOINT_LIMIT} rad (all 6 joints)")
    print(f"Gripper thresh: > {_GRIPPER_THRESH} → EM ON")
    print("\n  ✅ All tests passed.")


if __name__ == "__main__":
    main()
