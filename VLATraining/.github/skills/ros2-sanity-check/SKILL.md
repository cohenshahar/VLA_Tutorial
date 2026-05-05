---
name: ros2-sanity-check
description: "Diagnose and fix ROS2 setup problems for the VLA simulation project. Use when: ROS2 not found, colcon build fails, topics not visible, can't source workspace, node not starting, ros2 command not found, import error for rclpy, no topics listed, bridge not publishing, ros2 is broken, ROS2 setup, let's check ROS2."
argument-hint: "Optional: describe the specific ROS2 error you are seeing"
---

# ROS2 Sanity Check — VLA Tutorial

Run the canonical check sequence in order. Stop at the first failure and fix it before continuing.
This is Phase 9 territory: `VLATraining/vla_ws/src/mujoco_bridge/`

## When to Use
- Any ROS2 error or unexpected behavior
- Beginning Phase 9 work for the first time
- After system reboot (environment variables are not persistent)
- "ROS2 isn't working", "topics not showing up", "node won't start"

---

## Check Sequence

Run each check in order. **Stop at the first failure.**

### Check 1 — Is ROS2 installed?
```bash
ls /opt/ros/
```
Expected: `humble` directory exists.
If missing: ROS2 is not installed — stop, this is a system-level fix.

---

### Check 2 — Is the ROS2 underlay sourced?
```bash
printenv ROS_DISTRO
```
Expected output: `humble`

If blank or wrong:
```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO   # verify again
```

---

### Check 3 — Is the workspace built?
```bash
ls ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/
```
Expected: `mujoco_bridge/` directory present.

If missing or stale:
```bash
cd ~/Desktop/VLA_Tutorial/VLATraining/vla_ws
colcon build --packages-select mujoco_bridge
```
Watch for errors. Common ones:
- `CMakeLists.txt` missing a dependency → add to `find_package()`
- Python package not found → check `setup.py` entry points

---

### Check 4 — Is the workspace overlay sourced?
```bash
printenv AMENT_PREFIX_PATH | grep mujoco_bridge
```
Expected: path contains `mujoco_bridge`.

If missing:
```bash
source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
```

---

### Check 5 — Can Python import rclpy?
```bash
python3 -c "import rclpy; print('rclpy OK')"
```
Expected: `rclpy OK`

If fails with `ModuleNotFoundError`:
- The overlay is not sourced in this shell
- Or the venv is active and shadowing system Python — deactivate venv for ROS2 work:
  ```bash
  deactivate
  source /opt/ros/humble/setup.bash
  source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
  python3 -c "import rclpy; print('rclpy OK')"
  ```

> **Important:** rclpy lives in the system Python, not in the phase4 venv.
> For the bridge node, use `python3` (system), not `python` (venv).
> The bridge imports MuJoCo — install mujoco into system Python if needed:
> `pip3 install mujoco`

---

### Check 6 — Can the bridge node be found?
```bash
ros2 pkg list | grep mujoco_bridge
```
Expected: `mujoco_bridge`

If missing: workspace is not sourced or build failed — go back to Check 3.

---

### Check 7 — Run the bridge node
```bash
ros2 run mujoco_bridge bridge_node
```
Expected: node starts, prints status, no crash.

If it crashes immediately: read the traceback carefully. Common causes:
- Missing `world.xml` path — check the hardcoded path in the node
- `mujoco.MjModel.from_xml_path` fails — verify the XML path is absolute
- `rclpy.init()` called twice — ensure no duplicate init

---

### Check 8 — Verify topics are publishing
In a second terminal (with workspace sourced):
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
ros2 topic list
```
Expected Phase 9 topics:
```
/bridge/status
/joint_states
/ft_sensor
/cam_overhead/image_raw
/cam_side/image_raw
/cam_wrist/image_raw
/joint_commands
```

Check publish rate:
```bash
ros2 topic hz /joint_states   # should be ~100 Hz
ros2 topic echo /joint_states --once
```

---

## Common gotchas

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ros2: command not found` | underlay not sourced | `source /opt/ros/humble/setup.bash` |
| `No executable found` | workspace not built or sourced | rebuild + `source install/setup.bash` |
| `ModuleNotFoundError: rclpy` | venv is active | `deactivate` first |
| Topics not listed | node crashed silently | check terminal where node is running |
| Wrong topic Hz | sleep() in step loop too long | target 10ms per step for 100 Hz |
| XML path error | relative path | use `os.path.abspath(__file__)` to build path |

---

## Full source sequence (copy-paste ready)
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/VLA_Tutorial/VLATraining/vla_ws/install/setup.bash
cd ~/Desktop/VLA_Tutorial/VLATraining/sim
ros2 run mujoco_bridge bridge_node
```
