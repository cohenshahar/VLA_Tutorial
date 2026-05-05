# VLA Simulation — Complete Step-by-Step Instructions
**Author:** Shahar Cohen | BGU Mechatronics MSc
**Date:** 2026-04-28
**Version:** 1.0

---

## How to use this document

Each task has exactly three parts:
- **What to do** — a clear description of the action, written to give to a coding AI or follow yourself
- **Done when** — the exact thing you should see or verify before moving to the next task
- **Notes** — optional context or warnings

Work through the tasks in order. Do not skip ahead. Every task builds on the previous one.

When you give a task to a coding AI (Claude Code, Copilot, etc.), paste the full task text including the "What to do" section. The AI will write the code.

---

## Project structure (reference)

Every file you create goes inside one root folder. Call it `VLAResearch/sim/`. The structure will grow like this as you work:

```
VLAResearch/
  sim/
    assets/
      urdf/          ← KUKA URDF and mesh files go here
      mjcf/          ← converted MuJoCo XML files go here
    arm/             ← modular arm definition (swappable)
    scene/           ← table, box, cameras, sensors
    bridge/          ← ROS2 bridge node
    task_tree/       ← Task Tree Manager
    outputs/         ← saved images and videos
    main.py          ← orchestrator (runs everything together)
```

---

## Phase 0 — Environment Setup

---

### Task 0.1 — Verify Ubuntu version

**What to do:** Open a terminal. Run the command that prints the Ubuntu version. Make sure it is Ubuntu 22.04 or newer.

**Done when:** The terminal prints a version number of 22.04 or higher.

---

### Task 0.2 — Install system-level dependencies for MuJoCo

**What to do:** In the terminal, install the following Ubuntu packages: `libgl1-mesa-glx`, `libglib2.0-0`, `libosmesa6-dev`, `libglfw3`, `patchelf`, and `ffmpeg`. These are required for MuJoCo's rendering on Ubuntu.

**Done when:** The installation completes with no errors.

---

### Task 0.3 — Create the project folder structure

**What to do:** Create the root project folder `VLAResearch/sim/` in your home directory or preferred location. Inside it, create the following empty subfolders: `assets/urdf/`, `assets/mjcf/`, `arm/`, `scene/`, `bridge/`, `task_tree/`, `outputs/`. Each folder except `assets/` should contain an empty file named `__init__.py`.

**Done when:** You can open the `VLAResearch/sim/` folder in VS Code and see all subfolders listed in the file explorer.

---

### Task 0.4 — Create and activate a Python virtual environment

**What to do:** Inside the `VLAResearch/sim/` folder, create a Python virtual environment named `venv` using the latest available Python 3 version. After creating it, activate it in your terminal. Add the `venv/` folder to a `.gitignore` file in the project root.

**Done when:** Your terminal prompt shows `(venv)` at the beginning of the line.

---

### Task 0.5 — Install MuJoCo

**What to do:** With the venv activated, install the `mujoco` Python package using pip. Also install `numpy` and `matplotlib` at the same time.

**Done when:** The installation completes with no errors.

---

### Task 0.6 — Verify MuJoCo works

**What to do:** Create a Python script called `test_mujoco.py` in the `outputs/` folder. The script should load MuJoCo's built-in `humanoid.xml` model (it is included with the mujoco package — find the path using `mujoco.utils.find_base_path()`), run 100 physics steps, and print the simulation time after those steps.

**Done when:** Running the script prints a simulation time value (should be around 0.1 seconds) without any errors.

---

### Task 0.7 — Verify MuJoCo viewer opens

**What to do:** Modify the `test_mujoco.py` script to also open the MuJoCo passive viewer (the built-in 3D window). The viewer should show the humanoid model. You should be able to rotate the view with the mouse.

**Done when:** A window opens showing the humanoid robot in 3D. You can orbit around it with the mouse. Close the window to end the script.

---

### Task 0.8 — Install additional Python packages

**What to do:** With the venv active, install the following packages: `scipy`, `transforms3d`, `opencv-python`, `Pillow`, `rclpy` (the ROS2 Python client — note: this requires ROS2 to be sourced first, see Task 0.9), `sensor-msgs` and `geometry-msgs` from the ROS2 installation.

**Note:** For the ROS2 Python packages, you may need to install them via `apt` rather than pip (e.g. `ros-humble-rclpy`). Do whichever works on your system.

**Done when:** You can open a Python interpreter inside the venv and run `import mujoco`, `import numpy`, `import cv2`, `import PIL` — all without ImportError.

---

### Task 0.9 — Verify ROS2 Humble is active

**What to do:** Open a new terminal. Run the command that sources the ROS2 Humble setup file (the file is typically at `/opt/ros/humble/setup.bash`). Then run the command that lists all active ROS2 nodes — it should return an empty list since no nodes are running yet.

**Done when:** The `ros2 node list` command returns without error (empty output is fine).

**Note:** You will need to source ROS2 in every new terminal you open, unless you add the source command to your `~/.bashrc` file.

---

### Task 0.10 — Add ROS2 source to .bashrc

**What to do:** Add the command that sources `/opt/ros/humble/setup.bash` to the end of your `~/.bashrc` file. Also add the sourcing of your future workspace (we will create it in Task 0.12). After editing `.bashrc`, apply the changes to your current terminal.

**Done when:** Opening any new terminal automatically shows ROS2 as active — you can verify by running `echo $ROS_DISTRO` and seeing `humble` printed.

---

### Task 0.11 — Create a ROS2 workspace

**What to do:** Create a folder called `vla_ws` inside `VLAResearch/`. Inside it, create a `src/` subfolder. This is your ROS2 colcon workspace. Run the `colcon build` command inside `vla_ws/` to initialize it (it will succeed even with no packages yet).

**Done when:** Running `colcon build` inside `vla_ws/` completes with the message "Summary: 0 packages finished". Three new folders appear: `build/`, `install/`, `log/`.

---

### Task 0.12 — Open the project in VS Code

**What to do:** Open VS Code. Use "Open Folder" to open `VLAResearch/`. Install the following VS Code extensions if not already installed: Python, Pylance, ROS (by Microsoft). Set the Python interpreter to the venv you created in Task 0.4.

**Done when:** VS Code shows no red import errors when you hover over `import mujoco` in any Python file. The interpreter shown in the bottom bar says `venv`.

---

## Phase 1 — KUKA KR6 R900 URDF

---

### Task 1.1 — Download the KUKA KR6 support package

**What to do:** In the terminal, navigate to `VLAResearch/sim/assets/urdf/`. Clone the `kuka_experimental` repository from the `ros-industrial` GitHub organization. You only need the `kuka_kr6_support` subfolder — you can either clone the full repo and copy that folder, or use a sparse clone. The repo URL is `https://github.com/ros-industrial/kuka_experimental`.

**Done when:** Inside `assets/urdf/`, there is a folder called `kuka_kr6_support/` containing subfolders named `urdf/`, `meshes/`, and `launch/`.

---

### Task 1.2 — Inspect the URDF folder

**What to do:** Open `kuka_kr6_support/urdf/` in VS Code. Look at the files present. Find the file named `kr6r900sixx.xacro` (this is the robot description for the KR6 R900 sixx model). Open it and read its contents briefly — notice that it references joint names A1 through A6.

**Done when:** You can identify the file `kr6r900sixx.xacro` and confirm it contains references to joints named A1, A2, A3, A4, A5, A6.

---

### Task 1.3 — Install the xacro command-line tool

**What to do:** Install the `xacro` tool that converts XACRO files (parameterized URDF) into plain URDF XML. On Ubuntu with ROS2, install it via apt: `ros-humble-xacro`.

**Done when:** Running `xacro --version` in the terminal prints a version number without error.

---

### Task 1.4 — Convert the XACRO file to a plain URDF

**What to do:** In the terminal, navigate to `assets/urdf/kuka_kr6_support/urdf/`. Run the `xacro` command on `kr6r900sixx.xacro` and redirect the output to a new file called `kr6r900sixx.urdf` saved in `assets/urdf/`. The conversion must resolve all mesh paths correctly.

**Done when:** A file called `kr6r900sixx.urdf` exists in `assets/urdf/`. Opening it in VS Code shows it is valid XML starting with `<robot name=`.

---

### Task 1.5 — Inspect the converted URDF

**What to do:** Open `kr6r900sixx.urdf` in VS Code. Count the number of `<joint>` elements. Count the number of `<link>` elements. Write down the names of all 6 revolute joints.

**Done when:** You have identified exactly 6 revolute joints (A1–A6) and written their full names as they appear in the URDF file. Keep this list — you will need it later.

---

### Task 1.6 — Load the URDF into MuJoCo

**What to do:** Create a Python script called `arm/load_arm.py`. This script should define a function called `load_arm_model()` that takes the URDF file path as its only argument and returns a MuJoCo `MjModel` object loaded from that URDF. The function should print the number of degrees of freedom (`model.nq`) after loading.

This function is the modular arm interface — all other scripts will call `load_arm_model()` instead of loading the URDF directly. To swap the arm in the future, only this function needs to change.

**Done when:** Running the script directly (with a `__main__` block) prints a `nq` value of 6 without any error.

---

### Task 1.7 — Open the arm in the MuJoCo viewer

**What to do:** Extend the `arm/load_arm.py` script's `__main__` block to also open the MuJoCo passive viewer showing the loaded arm. The arm should appear in a 3D window. At this stage it will likely float in empty space with no ground — that is expected.

**Done when:** A window opens showing the KUKA arm in 3D. You can orbit with the mouse. The arm looks like a robotic arm (not a humanoid or anything unexpected).

---

### Task 1.8 — Verify all 6 joints are present

**What to do:** Extend the `arm/load_arm.py` script to also print the name of every joint in the model by iterating through `model.joint()` for each joint index from 0 to `model.njnt - 1`. Print each joint's name and type (revolute, free, etc.).

**Done when:** The printed list shows exactly 6 joints with the names A1–A6 and type `revolute` (or the MuJoCo equivalent integer). No extra joints appear.

---

### Task 1.9 — Verify joint limits

**What to do:** Extend the script to also print the lower and upper position limits for each joint (these come from `model.jnt_range`). Compare them to the KUKA KR6 R900 sixx datasheet values: A1 ±170°, A2 −190°/+45°, A3 −120°/+156°, A4 ±185°, A5 ±120°, A6 ±350°. Remember that MuJoCo stores limits in radians.

**Done when:** The printed limits, converted from radians to degrees, approximately match the datasheet values (within 1–2 degrees).

---

### Task 1.10 — Save the first render image

**What to do:** Extend the script to render one offscreen frame (without opening the viewer window) and save it as a PNG image to `outputs/arm_home_pose.png`. The image should show the arm from a 45° front-side angle. Use a resolution of 1280×720.

**Done when:** The file `outputs/arm_home_pose.png` exists and when opened shows the KUKA arm clearly, with all links visible, floating in an empty grey scene.

---

## Phase 2 — World Setup

---

### Task 2.1 — Create the world MJCF file

**What to do:** Create a new file called `scene/world.xml`. This will be a MuJoCo XML (MJCF) file that defines the world — not the robot, just the environment. For now, it should contain: a light source above the scene, a ground plane (flat, grey), and the sky color set to a neutral light blue. Include the arm URDF by referencing it from this world file.

**Note:** In MuJoCo MJCF, you include another XML using the `<include>` tag or the `<compiler meshdir>` attribute with a body reference. Look up MuJoCo's `<worldbody>` and `<include>` syntax.

**Done when:** Loading `scene/world.xml` in MuJoCo (via a Python script) opens the viewer showing the arm standing on a flat grey ground plane with correct lighting.

---

### Task 2.2 — Add a table

**What to do:** In `scene/world.xml`, add a table body. The table is a simple rectangular box geometry. Use these dimensions: 1.2 m wide, 0.8 m deep, 0.05 m thick (the tabletop slab). Place the tabletop surface at a height of 0.85 m above the ground. Give it a light wood-colour material (off-white or beige). The table should be a fixed body (no joints — it does not move).

**Done when:** The viewer shows the arm standing next to or above a rectangular table. The table does not fall through the floor.

---

### Task 2.3 — Position the arm on the table

**What to do:** In the world XML, move the arm's base position so that the arm is mounted on the centre-back edge of the table, with its base slightly inset. The arm's base plate should sit flush on the table surface (no floating, no sinking).

**Done when:** In the viewer, the arm's base sits cleanly on the table surface. The arm's reach extends over the table workspace. Save an updated render to `outputs/scene_arm_on_table.png`.

---

### Task 2.4 — Configure physics parameters

**What to do:** In `scene/world.xml`, set the physics options: timestep to 0.001 seconds (1 ms, giving 1000 Hz physics), gravity to −9.81 m/s² in the Z axis, and the solver to Newton (the default). Set `integrator` to `RK4` for better stability with robotic arms.

**Done when:** The simulation runs 1000 steps (simulating 1 second) in Python without any errors or NaN values in `data.qpos`.

---

### Task 2.5 — Verify physics step loop

**What to do:** Create a Python script called `scene/test_world.py`. It should load `scene/world.xml`, run 3000 physics steps (3 seconds of simulation), and after each 100 steps print the current simulation time and the arm's end-effector Z position (look up how to get a body's position by name from `data.xpos`). After the loop, open the viewer.

**Done when:** The script prints 30 lines of timestamps (0.1 s, 0.2 s, ..., 3.0 s). The end-effector Z position remains stable (does not drift or explode). The viewer shows the arm in its home pose.

---

## Phase 3 — Moving the Arm (Joint by Joint)

---

### Task 3.1 — Move joint A1 to +30 degrees

**What to do:** Create a Python script called `scene/test_joints.py`. It should load the world model, set joint A1's target position to +30 degrees (converted to radians), run the physics for 2 seconds (2000 steps), and then open the viewer showing the final pose. Save a render to `outputs/joint_A1_plus30.png`.

**Note:** In MuJoCo, you control a joint by setting `data.ctrl[i]` where `i` is the index of the actuator corresponding to that joint. First confirm what actuator names exist using `model.actuator(i).name` for each index.

**Done when:** The PNG shows the arm with A1 clearly rotated about 30 degrees from its default position. The rest of the arm follows naturally (it is a rigid body chain).

---

### Task 3.2 — Return A1 to home (0 degrees)

**What to do:** Extend `test_joints.py` to add a second phase: after reaching the +30° pose, set A1's target back to 0°, run 2000 more steps, and save a render to `outputs/joint_A1_return.png`.

**Done when:** The return render shows the arm back in its original home position (same as `outputs/arm_home_pose.png`).

---

### Task 3.3 — Move joint A2 to −30 degrees, then return

**What to do:** Add a test case for joint A2: target −30°, run 2000 steps, save render to `outputs/joint_A2_minus30.png`. Then return to 0°, run 2000 more steps.

**Done when:** The render shows A2 clearly moved — the arm's "shoulder" link has tilted backward or forward noticeably.

---

### Task 3.4 — Move joint A3 to +30 degrees, then return

**What to do:** Same procedure for A3. Save render to `outputs/joint_A3_plus30.png`.

**Done when:** The render shows the arm's elbow clearly bent upward.

---

### Task 3.5 — Move joint A4 to +45 degrees, then return

**What to do:** Same for A4. Save to `outputs/joint_A4_plus45.png`.

**Done when:** The render shows the forearm rotated around its long axis.

---

### Task 3.6 — Move joints A5 and A6, then return

**What to do:** Same for A5 (+30°) and then A6 (+90°), each separately. Save renders to `outputs/joint_A5_plus30.png` and `outputs/joint_A6_plus90.png`.

**Done when:** Both renders show the wrist joints clearly articulated.

---

### Task 3.7 — Move all joints to a reach pose simultaneously

**What to do:** Define a "reach pose" — a configuration where the arm extends forward and down, as if about to pick up an object from the table. Choose joint angles that place the end-effector roughly 30 cm in front of the arm base, 10 cm above the table surface. Set all 6 joints to these target values simultaneously and run the physics for 3 seconds. Save render to `outputs/reach_pose.png`.

**Note:** You may need to try a few angle combinations. It does not need to be exact — just a reachable forward pose. You will refine this later with proper inverse kinematics.

**Done when:** The render shows the arm clearly reaching forward over the table. The end-effector is visibly close to the table surface. Save the exact joint angle values you chose — you will need them later.

---

### Task 3.8 — Verify joint limit enforcement

**What to do:** Modify `test_joints.py` to try commanding A1 to 300 degrees (well beyond its ±170° limit). Run the physics and print the actual A1 joint position after 2 seconds.

**Done when:** The printed position is approximately ±170° (the limit), not 300°. MuJoCo enforces the joint limits automatically.

---

### Task 3.9 — Record a smooth trajectory video

**What to do:** Create a script that moves joint A1 through a full smooth sinusoidal oscillation (±60°) over 4 seconds. Render each step offscreen and save the frames as individual PNG files. Then use `ffmpeg` (installed in Task 0.2) to combine the frames into a video file at 30 fps. Save the video to `outputs/joint_A1_oscillation.mp4`.

**Done when:** The video file exists and plays correctly in any video player. The arm joint oscillates smoothly back and forth.

---

### Task 3.10 — Reach a target XYZ position using Jacobian IK

**What to do:** Add a function called `ik_jacobian(model, data, target_xyz, eef_body="link_6", max_iter=500, tol=0.005, step=0.5)` to `scene/test_joints.py`. The function should iteratively drive the end-effector to `target_xyz` using the Jacobian pseudoinverse method:

1. Call `mujoco.mj_forward(model, data)` to get the current EEF position from `data.xpos[eef_id]`.
2. Compute the position error: `pos_err = target_xyz - data.xpos[eef_id]`.
3. If `np.linalg.norm(pos_err) < tol`, stop (converged).
4. Compute the translational Jacobian with `mujoco.mj_jacBody(model, data, jac_p, None, eef_id)` — this gives a `(3, nv)` matrix. Take only the first 6 columns (the arm DOFs).
5. Solve for joint delta: `dq = np.linalg.lstsq(jac_arm, pos_err, rcond=None)[0]`.
6. Add `step * dq` to `data.qpos[:6]`, clamp each joint to its limit from `model.jnt_range`.
7. Repeat from step 1.

After convergence set `data.ctrl[:6] = data.qpos[:6]`, run 1000 physics steps to settle, then return the final `data.qpos[:6]`.

Add a test at the bottom of the script that calls `ik_jacobian` with `target_xyz = [0.4, 0.0, 0.95]` (centre of table workspace, 10 cm above surface), prints the achieved EEF position and the position error, and saves a render to `outputs/ik_target_reach.png`.

**Done when:** The printed position error is less than 5 mm. The render shows the arm extended to the target location. The function converges within the `max_iter` limit without NaN.

**Notes:**
- This is a pure kinematic solver — it moves `data.qpos` directly, not `data.ctrl`. The physics settle step at the end brings `data.ctrl` in sync so the arm holds the pose.
- The Jacobian pseudoinverse can get stuck near singularities (fully extended or folded arm). If it fails to converge, try a different starting pose or reduce the `step` size to 0.3.
- This method only controls position (3 DOF). Orientation of the end-effector is not constrained — it will land at whatever orientation the kinematics produce.

---

## Phase 4 — Electromagnetic End-Effector

---

### Task 4.1 — Add the EM pad geometry to the A6 flange

**What to do:** In `scene/world.xml` (or a new include file `scene/em_effector.xml`), add a cylindrical body attached to the arm's final link (A6 or the flange link — use the exact link name from the URDF you found in Task 1.5). The cylinder should represent the EM pad: diameter 8 cm, height 2 cm, coloured dark grey. It should be rigidly attached — no joint between it and the flange.

**Done when:** The viewer shows a flat cylinder attached to the end of the arm. It moves with the arm when joints change. Save a render to `outputs/em_pad_attached.png`.

---

### Task 4.2 — Add an attachment site on the EM pad

**What to do:** Add a MuJoCo `<site>` element at the bottom face of the EM pad cylinder. Name it `em_contact_site`. This site will be used as the attachment point for the weld constraint later. Sites are invisible by default (used as reference points, not visual geometry).

**Done when:** In Python, you can look up `model.site('em_contact_site')` without error, confirming the site exists in the model.

---

### Task 4.3 — Verify the EM pad moves with the arm

**What to do:** Write a small Python test that moves the arm to the reach pose (Task 3.7), then prints the world position of `em_contact_site`. Also print the position when the arm is at home pose. Confirm they are different values.

**Done when:** The two printed positions are clearly different, confirming the site position updates correctly with arm movement.

---

### Task 4.4 — Save a composite render showing the EM pad from two angles

**What to do:** Render the arm in the reach pose from two camera angles: front view and top-down view. Save both to `outputs/em_pad_frontview.png` and `outputs/em_pad_topview.png`.

**Done when:** Both images clearly show the grey EM cylinder at the end of the arm.

---

## Phase 5 — Scene Objects (Box and Target Zone)

---

### Task 5.1 — Add the metal box as a free body

**What to do:** In the world XML, add a rectangular box body with a `freejoint` (this allows it to move freely under gravity). Initial dimensions: 10 cm × 10 cm × 10 cm. Give it a metallic silver-grey colour. Set its initial position to the centre of the table, 5 cm above the table surface (so it will fall and land on the table when physics starts).

**Done when:** When the physics simulation starts and runs for 0.5 seconds, the box falls from its initial position and lands stably on the table surface. It does not fall through the table.

---

### Task 5.2 — Configure box mass and inertia

**What to do:** Set the box's mass to 0.5 kg. MuJoCo can automatically compute the inertia tensor from the geometry and mass — use this automatic computation (set `diaginertia` to auto or use `<inertial pos="0 0 0" mass="0.5"/>`). Leave the density-based inertia as MuJoCo default.

**Done when:** Printing `model.body_mass[box_body_id]` from Python shows approximately 0.5. The box still lands stably.

---

### Task 5.3 — Configure friction for box and table contact

**What to do:** Set the friction coefficients for the box-table contact. Use these values: sliding friction 0.8, torsional friction 0.005, rolling friction 0.0001. These values are set on the box's geom using MuJoCo's `<geom friction="...">` attribute.

**Done when:** When you push the box in the simulation by applying a brief lateral force (set `data.xfrc_applied` on the box body for a few steps), the box slides a bit and then stops — it does not slide endlessly (too slippery) and does not immediately freeze (too sticky).

---

### Task 5.4 — Apply a test impulse to the box

**What to do:** Write a test script that loads the scene, waits for the box to settle on the table (run 500 steps), then applies a short impulse force of 5 N in the X direction for 50 steps, then removes the force and runs 1000 more steps. Print the box position every 100 steps. Save a video to `outputs/box_impulse_test.mp4`.

**Done when:** The printed positions show the box moving when the force is applied, then decelerating and stopping due to friction. The video confirms this.

---

### Task 5.5 — Add the target zone

**What to do:** Add a flat rectangular marker on the table surface to represent the target zone where the box must be placed. Make it a thin box (2 cm tall, 15 cm × 15 cm) with a bright orange or yellow colour. Place it on the opposite side of the table from the arm's initial reach position. It is a fixed body (no joint).

**Done when:** The viewer shows the coloured target zone square on the table. It is clearly visible and distinguishable from the table surface.

---

### Task 5.6 — Add a contact sensor at the target zone

**What to do:** Add a `<sensor type="touch">` element on the target zone geom. Name it `target_touch_sensor`. This sensor outputs 1 when something is in contact with the target zone geom, and 0 otherwise.

**Done when:** In a Python test: load the scene, place the box directly above the target zone, run physics until the box settles, then read `data.sensordata` at the target touch sensor index. It should read a positive value. Move the box away (set its position far from the target) and verify the sensor reads 0.

---

### Task 5.7 — Render the full scene (arm + box + target)

**What to do:** Render the complete scene from a 45° front angle showing all elements: KUKA arm in home pose, box sitting on the table, target zone marker visible. Save to `outputs/full_scene.png`.

**Done when:** The image clearly shows all three elements together. The lighting is good enough to distinguish the arm, box, and target zone by colour.

---

## Phase 6 — Electromagnetic Attachment System

---

### Task 6.1 — Define the weld equality constraint

**What to do:** Add a `<equality>` element of type `weld` to the world XML. It should connect the `em_contact_site` (on the arm) to a site on the top face of the box (add a site called `box_top_site` to the box body first). Set the constraint as initially inactive by setting `active="false"`.

**Done when:** The model loads without error. In Python, you can find the constraint by name using `model.equality('em_weld')` (or whatever you named it). The box stays on the table when physics runs — the inactive constraint has no effect.

---

### Task 6.2 — Activate the constraint manually and observe

**What to do:** Write a Python test that: loads the scene, moves the arm to the reach pose (Task 3.7) so the EM pad is directly above the box, then activates the weld constraint by setting `model.eq_active[weld_id] = True`. Run 2000 more physics steps and print the box position every 200 steps.

**Done when:** After activation, the box position follows the arm. If the arm was hovering above the table, the box should lift off the table and move with the end-effector.

---

### Task 6.3 — Move the arm while holding the box

**What to do:** Extend the test: after activating the constraint and confirming the box is attached, command the arm to move to a second pose (e.g., A1 rotated 45° from the reach pose). Run 2000 steps. Print box position before and after the move.

**Done when:** The box position changes corresponding to the arm movement. The box is clearly being carried by the arm.

---

### Task 6.4 — Deactivate the constraint and observe the drop

**What to do:** Extend the test: after the arm has moved with the box, deactivate the constraint by setting `model.eq_active[weld_id] = False`. Run 1000 more steps and print box position.

**Done when:** After deactivation, the box falls due to gravity and lands on the table (or the floor if the arm moved it away from the table). The box position changes sharply downward after deactivation.

---

### Task 6.5 — Implement the proximity pre-condition

**What to do:** Create a Python function called `em_can_activate(data, model, threshold_m=0.025)` in a file called `scene/em_controller.py`. This function should return `True` only if the distance between `em_contact_site` and `box_top_site` is less than `threshold_m` (2.5 cm). It reads the positions of both sites from `data.site_xpos` and computes the Euclidean distance.

**Done when:** In a Python test, call this function with the arm in the reach pose (close to the box) — it returns `True`. Call it with the arm at home pose (far from the box) — it returns `False`.

---

### Task 6.6 — Implement the full EM activate/deactivate cycle

**What to do:** In `scene/em_controller.py`, create two more functions: `em_activate(model, data)` which checks proximity (using the function from 6.5) and if True sets `model.eq_active[weld_id] = True` and returns `True`; and `em_deactivate(model)` which unconditionally sets the constraint inactive and returns `True`.

**Done when:** In a combined test script: arm starts at home (EM activate request → returns False, box stays on table). Arm moves to reach pose (EM activate request → returns True, box lifts). Arm moves to target zone. EM deactivate called (box drops to target zone area). Print final box position and confirm it is near the target zone.

---

### Task 6.7 — Record a full EM cycle video

**What to do:** Script the complete sequence: home → reach → activate EM → lift → move to target → deactivate EM → box drops. Render every physics step offscreen and save as a video to `outputs/em_full_cycle.mp4`.

**Done when:** The video plays and shows the arm approaching the box, the box attaching and lifting, the arm transporting the box over the target zone, and the box dropping. The video is smooth (no jumps or glitches at the constraint activation moment).

---

## Phase 7 — Sensor Suite

---

### Task 7.1 — Add joint position sensors

**What to do:** In the world XML, add 6 `<sensor type="jointpos">` elements, one for each joint A1–A6. Name them `sensor_pos_A1` through `sensor_pos_A6`. These will appear sequentially in the `data.sensordata` array.

**Done when:** In Python, after running one step, read the sensordata array. The first 6 values should match `data.qpos` for the 6 joints (they should be identical or nearly identical).

---

### Task 7.2 — Add joint velocity sensors

**What to do:** Add 6 `<sensor type="jointvel">` elements, one per joint, named `sensor_vel_A1` through `sensor_vel_A6`.

**Done when:** When the arm is moving, these sensor values are non-zero. When the arm is stationary, they read approximately 0.

---

### Task 7.3 — Add joint torque (actuator force) sensors

**What to do:** Add 6 `<sensor type="actuatorfrc">` elements, one per actuator (the motor driving each joint), named `sensor_torque_A1` through `sensor_torque_A6`.

**Done when:** When the arm holds a pose against gravity with the box attached, the torque sensors show non-zero values. The shoulder joints (A1, A2) should show higher torques than the wrist joints (A5, A6).

---

### Task 7.4 — Add a 6-axis force/torque sensor at the end-effector

**What to do:** Add two sensors to the EM pad body: one `<sensor type="force">` and one `<sensor type="torque">`, both attached to the `em_contact_site`. Name them `sensor_ee_force` and `sensor_ee_torque`. Each produces a 3-element vector (x, y, z).

**Done when:** When the arm is holding the box (EM active, arm stationary), the force sensor Z component reads approximately 4.9 N (the weight of the 0.5 kg box under 9.81 m/s² gravity). When no box is attached, the force sensor reads approximately 0 in all components.

---

### Task 7.5 — Add a proximity rangefinder sensor

**What to do:** Add a `<sensor type="rangefinder">` at the `em_contact_site`, pointing in the downward direction (in the end-effector's local frame). Name it `sensor_proximity`. This sensor measures distance to the nearest surface in the specified direction.

**Done when:** When the arm is in the reach pose hovering 5 cm above the box, the sensor reads approximately 0.05 (5 cm). When the arm is at home pose far from the table, the sensor reads a large value (or the max range if configured). Confirm the values make sense by comparing to the known arm position.

---

### Task 7.6 — Create a sensor logger

**What to do:** Create a Python class called `SensorLogger` in `scene/sensor_logger.py`. This class should have a method called `log_step(step, data, model)` that reads all sensor values defined in Tasks 7.1–7.5 plus the target touch sensor (Task 5.6) and appends a row to an internal list. Each row contains: step number, simulation time, joint positions (6), joint velocities (6), joint torques (6), EE force (3), EE torque (3), proximity (1), target touch (1) — 27 values per row. The class should also have a `save(filename)` method that writes all logged rows to a CSV file.

**Done when:** Run the full EM cycle script from Task 6.6 with the logger active. After the run, call `save('outputs/sensor_log_test.csv')`. Open the CSV and verify it has 27 columns and one row per simulation step. The F/T sensor column shows ~4.9 N during the box-carrying phase.

---

## Phase 8 — Camera Setup

---

### Task 8.1 — Add the overhead camera

**What to do:** In the world XML, add a `<camera>` element attached to the world body (fixed in space). Name it `cam_overhead`. Position it 1.5 m directly above the centre of the table workspace, pointing straight down (negative Z direction). Set the field of view to 60 degrees.

**Done when:** In Python, you can reference the camera by name using `mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'cam_overhead')` without error.

---

### Task 8.2 — Render a frame from the overhead camera

**What to do:** Write a Python function called `render_camera(model, data, cam_name, width, height)` in `scene/camera_utils.py`. It should set up an offscreen rendering context, render the specified camera, and return a numpy array of shape `(height, width, 3)` with RGB values. Call it with `cam_overhead`, width=640, height=480. Save the result as `outputs/cam_overhead_test.png`.

**Done when:** The PNG shows a top-down view of the table with the arm and box visible from above. The target zone marker is visible. The image is not black or garbled.

---

### Task 8.3 — Verify the overhead camera covers the full workspace

**What to do:** Move the box to three different positions on the table (left edge, centre, right edge) and render the overhead camera at each position. Save as `outputs/cam_overhead_left.png`, `outputs/cam_overhead_centre.png`, `outputs/cam_overhead_right.png`.

**Done when:** In all three images, the box is fully visible and not cut off by the camera's field of view. If the box goes out of frame in any position, adjust the camera height or FOV and repeat.

---

### Task 8.4 — Add the side camera

**What to do:** Add a second `<camera>` named `cam_side` in the world XML. Place it 1.2 m to one side of the table and 0.8 m above the table surface, angled at roughly 30–45° downward toward the centre of the workspace. Set FOV to 60 degrees.

**Done when:** Rendering `cam_side` produces a `outputs/cam_side_test.png` that shows the arm, table, and box from a side/angled perspective. The table surface is visible. The arm occupies roughly the centre of the frame.

---

### Task 8.5 — Verify the side camera shows depth information visually

**What to do:** Place the box at three different distances from the arm (near, middle, far on the table) and render the side camera at each position. Save as `outputs/cam_side_near.png`, `outputs/cam_side_mid.png`, `outputs/cam_side_far.png`.

**Done when:** The three images clearly show the box at visibly different positions — the side camera gives useful depth perspective that the overhead camera does not.

---

### Task 8.6 — Add the wrist camera

**What to do:** Add a third `<camera>` named `cam_wrist` attached to the EM pad body (the same body added in Task 4.1). Position it on the top face of the EM pad pointing forward and slightly downward (so it sees what is in front of the end-effector). Set FOV to 80 degrees (wide-angle, typical for wrist cameras).

**Done when:** Rendering `cam_wrist` shows a first-person perspective from the end-effector. When the arm is at the reach pose hovering above the box, the box is visible in the centre of the frame.

---

### Task 8.7 — Verify the wrist camera moves with the arm

**What to do:** Render the wrist camera at three arm poses: home pose, reach pose, and A1 rotated 90°. Save as `outputs/cam_wrist_home.png`, `outputs/cam_wrist_reach.png`, `outputs/cam_wrist_rotated.png`.

**Done when:** The three images show clearly different views — the camera field of view changes with the arm pose. In the reach pose, the box is visible and centred.

---

### Task 8.8 — Create a camera publisher utility

**What to do:** In `scene/camera_utils.py`, add a class called `CameraPublisher` that holds a list of camera names and their target frequencies. It should have a method `get_frames(model, data)` that returns a dictionary mapping each camera name to its current rendered RGB numpy array. This class will be used later to feed frames into ROS2 topics.

**Done when:** Calling `get_frames()` returns a dictionary with 3 keys (`cam_overhead`, `cam_side`, `cam_wrist`), each mapping to a numpy array of shape `(480, 640, 3)`.

---

### Task 8.9 — Record a multi-camera video of the EM cycle

**What to do:** Re-run the EM full cycle from Task 6.7, but this time render all 3 cameras at every step and save each camera's frames as a separate video. Combine the three videos side-by-side (using ffmpeg's hstack filter) into a single wide video saved to `outputs/multicam_em_cycle.mp4`.

**Done when:** The video shows three panels side-by-side (overhead, side, wrist) all showing the same EM pick-and-place cycle from different angles simultaneously.

---

## Phase 9 — ROS2 Bridge

---

### Task 9.1 — Create the ROS2 package

**What to do:** Navigate to `VLAResearch/vla_ws/src/`. Create a new ROS2 Python package called `mujoco_bridge` using the `ros2 pkg create` command with `--build-type ament_python`. Run `colcon build` in `vla_ws/` after creating it.

**Done when:** Running `ros2 pkg list` shows `mujoco_bridge` in the output.

---

### Task 9.2 — Write and test a Hello World ROS2 node

**What to do:** Inside `mujoco_bridge/`, create a simple Python node that publishes the string "MuJoCo bridge alive" on a topic called `/bridge/status` at 1 Hz. Build the package and run the node.

**Done when:** In a second terminal, running `ros2 topic echo /bridge/status` shows the message "MuJoCo bridge alive" appearing once per second.

---

### Task 9.3 — Publish joint states from MuJoCo

**What to do:** Extend the bridge node to read `data.qpos` from the running MuJoCo simulation (6 values) and publish them as a `sensor_msgs/JointState` message on `/joint_states`. The message should contain joint names (A1–A6), positions, velocities, and efforts (torques). Publish at 100 Hz.

**Done when:** Running `ros2 topic echo /joint_states` in a second terminal shows messages appearing at roughly 100 Hz with 6 joint names and 6 position values. Running `ros2 topic hz /joint_states` shows approximately 100 Hz.

---

### Task 9.4 — Publish joint torques

**What to do:** Publish the actuator force sensor values (from Task 7.3) as a second `sensor_msgs/JointState` message on `/joint_torques`. Use the same joint names.

**Done when:** `ros2 topic echo /joint_torques` shows messages with torque values that change when the arm's load changes.

---

### Task 9.5 — Subscribe to joint commands

**What to do:** Add a subscriber to the bridge node listening to `/joint_commands` (message type `trajectory_msgs/JointTrajectory`). When a message is received, the bridge should read the target joint positions from the first trajectory point and update `data.ctrl` accordingly (commanding the arm actuators).

**Done when:** From a second terminal, publish a `/joint_commands` message commanding A1 to 30 degrees. Confirm the arm moves in the MuJoCo viewer. Running `ros2 topic echo /joint_states` should then show the updated A1 position.

---

### Task 9.6 — Publish the F/T sensor

**What to do:** Publish the end-effector force sensor data as a `geometry_msgs/WrenchStamped` message on `/ft_sensor`. Publish at 100 Hz.

**Done when:** `ros2 topic echo /ft_sensor` shows messages. When the arm holds the box (EM active), the `force.z` field reads approximately −4.9 (downward load from box weight).

---

### Task 9.7 — Publish the proximity sensor

**What to do:** Publish the rangefinder value as a `sensor_msgs/Range` message on `/proximity`. Publish at 50 Hz. Set the min range to 0.001 m and max range to 1.0 m.

**Done when:** `ros2 topic echo /proximity` shows messages. Move the arm close to the box and observe the range value decrease in the terminal.

---

### Task 9.8 — Publish the EM state

**What to do:** Publish the current EM activation state (True/False) as a `std_msgs/Bool` message on `/em_state`. Publish on every change (not at a fixed rate — only when the state changes).

**Done when:** Monitor `/em_state` with `ros2 topic echo`. Activate the EM manually → topic shows `True`. Deactivate → topic shows `False`.

---

### Task 9.9 — Publish target contact

**What to do:** Publish the target touch sensor value (from Task 5.6) as a `std_msgs/Bool` message on `/target_contact`. Publish at 50 Hz.

**Done when:** Place the box on the target zone in simulation → `/target_contact` shows `True`. Box elsewhere → shows `False`.

---

### Task 9.10 — Publish the overhead camera image

**What to do:** Publish the overhead camera frames as `sensor_msgs/Image` messages on `/camera/overhead/image_raw`. Use the `cv_bridge` library to convert the numpy RGB array to a ROS2 Image message. Publish at 6 Hz.

**Done when:** `ros2 topic hz /camera/overhead/image_raw` shows approximately 6 Hz. `ros2 topic echo /camera/overhead/image_raw --no-arr` shows messages with correct width (640), height (480), and encoding (`rgb8`).

---

### Task 9.11 — Publish the side camera image

**What to do:** Same as Task 9.10 but for the side camera on topic `/camera/side/image_raw`.

**Done when:** Both camera topics publish simultaneously at 6 Hz each.

---

### Task 9.12 — Publish the wrist camera image

**What to do:** Same for the wrist camera on `/camera/wrist/image_raw`.

**Done when:** All three camera topics are live simultaneously. `ros2 topic list` shows all three.

---

### Task 9.13 — Run ros2 topic list and verify all topics

**What to do:** With the full bridge running, run `ros2 topic list` in a terminal and copy the full output.

**Done when:** The list contains exactly these topics (and possibly `/bridge/status`):
- `/joint_states`
- `/joint_torques`
- `/joint_commands`
- `/ft_sensor`
- `/proximity`
- `/em_state`
- `/target_contact`
- `/camera/overhead/image_raw`
- `/camera/side/image_raw`
- `/camera/wrist/image_raw`
- `/task_tree/status` (this one will be added in Phase 10)

---

### Task 9.14 — Run ros2 topic hz on all topics and record rates

**What to do:** Run `ros2 topic hz` separately on each topic and record the measured Hz. Write the results in a text file saved to `outputs/topic_rates.txt`.

**Done when:** The file exists. All rates are within 10% of their targets (joint_states ≈100 Hz, cameras ≈6 Hz, ft_sensor ≈100 Hz, proximity ≈50 Hz).

---

## Phase 10 — Task Tree Manager

---

### Task 10.1 — Define the TaskNode data structure

**What to do:** Create a file `task_tree/task_node.py`. Define a Python dataclass (or simple class) called `TaskNode` with these fields: `name` (string), `language_instruction` (string — what the VLA will be told to do for this subtask), `primitives` (list of strings describing the low-level actions), `postcondition_fn` (a callable that takes current sensor readings and returns True/False), and `status` (one of: "pending", "in_progress", "completed", "failed").

**Done when:** You can create a `TaskNode` in Python, set all its fields, and call `postcondition_fn` passing a mock sensor reading dictionary. No import errors.

---

### Task 10.2 — Define the root Task node

**What to do:** In a new file `task_tree/pick_and_place_tree.py`, create the root task node. Name: "EM Pick-and-Place". Language instruction: "Pick up the metal box from the table and place it on the orange target zone." Primitives: ["approach_box", "attach_em", "lift_box", "transport_to_target", "release_box"]. Postcondition: always returns True (the task is complete when all subtasks complete).

**Done when:** The root TaskNode is created without error.

---

### Task 10.3 — Define Subtask 1: Approach

**What to do:** In the same file, define a child TaskNode for the approach subtask. Name: "Approach". Language instruction: "Move to the box and position the end-effector above it." Primitives: ["move_to_preapproach_height", "lower_above_box"]. Postcondition function: returns True when proximity sensor reads less than 0.025 m (2.5 cm) — the arm is close to the box.

**Done when:** In a test, pass a sensor reading dictionary with `{"proximity": 0.02}` — function returns True. Pass `{"proximity": 0.1}` — returns False.

---

### Task 10.4 — Define Subtask 2: Attach and Lift

**What to do:** Define a second child TaskNode. Name: "Attach and Lift". Language instruction: "Activate the electromagnet to pick up the box, then lift it." Primitives: ["activate_em", "verify_attachment", "lift_to_carry_height"]. Postcondition: returns True when `em_state == True` AND `ee_force_z > 4.0` (box is held) AND box Z position is more than 0.1 m above the table surface.

**Done when:** Test with `{"em_state": True, "ee_force_z": 4.9, "box_z": 0.95}` (table height ~0.85 m, so box is 0.1 m above) → True. Test with `{"em_state": False, "ee_force_z": 0.0, "box_z": 0.85}` → False.

---

### Task 10.5 — Define Subtask 3: Transport and Release

**What to do:** Define the third child TaskNode. Name: "Transport and Release". Language instruction: "Carry the box to the orange target zone and release it." Primitives: ["move_to_target_zone", "lower_box", "deactivate_em"]. Postcondition: returns True when `target_contact == True` AND `em_state == False`.

**Done when:** Test with `{"target_contact": True, "em_state": False}` → True. Any other combination → False.

---

### Task 10.6 — Implement the TaskTreeManager

**What to do:** Create a class called `TaskTreeManager` in `task_tree/task_tree_manager.py`. It takes a root TaskNode and a list of child TaskNodes (in order). It has a method `step(sensor_readings)` that: checks the postcondition of the current active subtask; if True, marks that subtask as completed and advances to the next subtask; if the last subtask completes, marks the root task as completed. It also has a method `get_current_instruction()` returning the language instruction of the currently active subtask. It has a method `get_status_dict()` returning a dictionary with current subtask name, status of all subtasks, and overall completion percentage.

**Done when:** In a Python test, manually call `step()` with sensor readings that satisfy each postcondition in sequence. After 3 calls (one per subtask), `get_status_dict()` shows all subtasks completed and 100% completion.

---

### Task 10.7 — Connect TaskTreeManager to the sensor bus

**What to do:** Modify `TaskTreeManager` so that instead of requiring sensor readings to be passed manually, it can accept a `SensorLogger` instance (from Task 7.6) and automatically read the latest sensor values from it on each `step()` call. Add a method `step_from_logger(logger)`.

**Done when:** In a full simulation run, the manager can read live sensor data and advance subtasks based on what is actually happening in the simulation — not hardcoded values.

---

### Task 10.8 — Publish task tree status to ROS2

**What to do:** Add a ROS2 publisher to the bridge node that publishes the output of `task_tree_manager.get_status_dict()` as a JSON string in a `std_msgs/String` message on `/task_tree/status`. Publish whenever the status changes (not at a fixed rate).

**Done when:** Run the simulation through a scripted EM cycle. Monitor `/task_tree/status` with `ros2 topic echo`. You should see exactly 3 messages appear (one when each subtask starts, or when it completes — choose whichever is more informative).

---

### Task 10.9 — Test postcondition 1 with scripted arm movement

**What to do:** Script the arm to move from home pose to the reach pose (hovering close to the box). Log sensor data throughout. After the arm reaches the target, check if the Subtask 1 postcondition fires. Print the proximity sensor value at each step of the approach.

**Done when:** The proximity sensor value decreases as the arm approaches, eventually drops below 0.025 m, and the TaskTreeManager automatically transitions from Subtask 1 to Subtask 2 at that moment.

---

### Task 10.10 — Test postcondition 2 with scripted EM activation and lift

**What to do:** Continue from Task 10.9. Script: activate EM, run 200 steps (box attaches), command arm to lift 0.15 m upward. Log F/T and box Z position throughout. After lifting, check if Subtask 2 postcondition fires.

**Done when:** The F/T sensor reads > 4.0 N after box attaches, the box Z rises above table + 0.1 m after lift, and the TaskTreeManager transitions to Subtask 3 automatically.

---

### Task 10.11 — Test postcondition 3 with scripted transport and release

**What to do:** Continue from Task 10.10. Script: move arm over target zone, lower arm 0.1 m, deactivate EM. Run 500 steps (box drops and settles). Check if Subtask 3 postcondition fires.

**Done when:** The target touch sensor reads True after the box lands on the target zone, EM state is False, and the TaskTreeManager marks all subtasks completed.

---

### Task 10.12 — Run and record the full scripted task sequence

**What to do:** Combine Tasks 10.9–10.11 into a single clean run script called `task_tree/run_scripted_sequence.py`. Run it and record the complete multi-camera video of the full task, with sensor data overlaid as text on the overhead camera view (current subtask name, proximity reading, F/T reading, EM state). Save to `outputs/full_task_scripted.mp4`.

**Done when:** The video shows the complete pick-and-place sequence from start to finish with sensor data visible. The task tree status transitions are reflected in the overlaid text.

---

## Phase 11 — OpenVLA on Kaggle (GPU Required)

---

### Task 11.1 — Set up a Kaggle notebook

**What to do:** Log in to Kaggle. Create a new notebook. Set the accelerator to GPU (T4 × 1). In the notebook settings, enable internet access. Name the notebook `openvla-inference-test`.

**Done when:** The notebook is open and shows "GPU" in the accelerator indicator. The notebook can access the internet (run `!ping -c 1 google.com` in a cell).

---

### Task 11.2 — Install OpenVLA dependencies

**What to do:** In the first notebook cell, install: `transformers` (latest), `accelerate`, `timm`, `torch` (should be preinstalled on Kaggle), and `huggingface_hub`. Also install `Pillow` and `numpy`.

**Done when:** All imports succeed without error in the next cell: `import transformers`, `import torch`, `import PIL`.

---

### Task 11.3 — Load the OpenVLA model

**What to do:** In a new cell, load the OpenVLA model from HuggingFace: model ID is `openvla/openvla-7b`. Load both the processor and the model. Use `torch.float16` (half precision) to reduce VRAM usage. Move the model to the GPU.

**Note:** This download is approximately 14 GB. It will take several minutes on first run.

**Done when:** The model loads without error. Running `print(model.device)` shows `cuda`. Running `print(torch.cuda.memory_allocated() / 1e9)` shows a value between 10 and 16 GB.

---

### Task 11.4 — Run a single inference with a test image

**What to do:** Create a test image: a 224×224 RGB image of a random scene (even a solid colour is fine for this test). Create a test instruction string: "Pick up the box and place it on the target." Pass both through the processor and model, and generate the output action tokens.

**Done when:** The model produces an output without error. The output can be decoded by the processor into an action array. Print the raw output tokens.

---

### Task 11.5 — Verify the action output shape

**What to do:** Decode the model output into a numpy action array. Print its shape and values.

**Done when:** The output is a numpy array of shape `(7,)` — 6 joint deltas and 1 gripper value. The values are in the range approximately −1 to +1 before unnormalization.

---

### Task 11.6 — Benchmark inference speed

**What to do:** Run 10 consecutive inferences with the same test image and instruction. Time each one using Python's `time.time()`. Print the mean and standard deviation of the inference times.

**Done when:** The mean inference time is printed. Note the value — this determines how many Hz the VLA can run at. On a Kaggle T4, typical values are 1–3 seconds per inference.

---

### Task 11.7 — Test with a real camera frame

**What to do:** Take the wrist camera frame from `outputs/cam_wrist_reach.png` (saved in Task 8.6), upload it to the Kaggle notebook, load it as a PIL image, and run inference with the instruction "Pick up the metal box." Print and inspect the action output.

**Done when:** The inference completes. The action output is a 7-element array with values that look physically meaningful (small joint deltas, not extreme values).

---

### Task 11.8 — Document the Kaggle → local bridge plan

**What to do:** Write a short text document (in `outputs/kaggle_bridge_plan.txt`) describing how OpenVLA on Kaggle will communicate with the local simulation: the Kaggle notebook will expose a simple HTTP endpoint using `flask` or `fastapi`; the local simulation sends a camera frame and receives action tokens in response. This bridge will be implemented in a later phase.

**Done when:** The file exists and describes the interface clearly enough to implement from.

---

## Summary checklist

After completing all tasks, verify:

- [ ] MuJoCo loads KUKA KR6 R900 sixx arm from URDF
- [ ] Arm moves all 6 joints correctly with joint limit enforcement
- [ ] EM end-effector picks up and releases the box reliably
- [ ] All sensors log correct values (F/T reads box weight, proximity reads distance)
- [ ] All 3 cameras render correct views at ≥6 Hz
- [ ] All ROS2 topics are live and at correct frequencies
- [ ] Task Tree Manager transitions through all 3 subtasks correctly
- [ ] Full scripted pick-and-place video recorded
- [ ] OpenVLA runs on Kaggle and produces 7-element action output

---

## Appendix — Uploading your work to GitHub

The project is hosted at: **https://github.com/cohenshahar/VLA_Tutorial**

### One-time setup (first time only)

```bash
# 1. Make sure git is installed
git --version

# 2. Set your identity (only needed once per machine)
git config --global user.name  "Your Name"
git config --global user.email "you@example.com"

# 3. Authenticate with GitHub
#    Option A — GitHub CLI (recommended, installs once)
sudo apt install gh
gh auth login        # follow the prompts; choose HTTPS + browser

#    Option B — Personal Access Token (classic)
#    Go to GitHub → Settings → Developer settings → Personal access tokens → Generate new token
#    Scope: repo (full control). Copy the token.
#    When git asks for a password, paste the token.
```

### Uploading after each session

Run these commands from inside the `VLATraining/` folder:

```bash
cd /home/shahar/Desktop/phase4/VLATraining

# 1. Stage everything that changed
git add -A

# 2. Commit with a short description of what you did
git commit -m "Phase X: brief description of changes"

# 3. Push to GitHub
git push origin main
```

### Checking what will be committed

```bash
git status          # shows modified / new / deleted files
git diff --stat     # shows a summary of line changes per file
git log --oneline -5   # shows the last 5 commits
```

### If the push is rejected (someone else pushed first)

```bash
git pull --rebase origin main   # bring in their changes, replay yours on top
git push origin main
```

### What NOT to upload (already in .gitignore)

- `phase4_env/` — the Python virtual environment (recreate with `pip install -r requirements.txt`)
- `outputs/*.mp4` and `outputs/*.png` — large render files (add them manually if you want)
- `__pycache__/` — Python bytecode cache

---

*VLA Research | Shahar Cohen | BGU Mechatronics | 2026-04-28*
*Next after completion: connect OpenVLA output to Task Tree Manager and replace scripted policy with VLA-generated actions.*
