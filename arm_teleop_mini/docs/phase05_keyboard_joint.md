# Phase 05 — Keyboard teleop — joint mode

**Goal**: Holding `q` rotates joint 1 positive; releasing stops it within the deadman timeout.

**Concepts introduced**:
- `curses` non-blocking key polling in a background thread
- Deadman timeout pattern
- Publishing at a fixed rate regardless of key events

**Deliverables**:
- `arm_teleop/keyboard_teleop_node.py` — `_read_keys`, `_publish_cmd` implemented for joint mode

**Smoke test**:
- Command: run `keyboard_teleop_node`, hold `q`
- Expected: joint 1 rotates in viewer; release `q`, joint stops within 200 ms

**Notes** (filled in as we go):
