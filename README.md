# Catalyst — ROS 2 control + vision helper scripts

This repo is a small collection of **ROS 2 (rclpy) helper nodes** used around the *Catalyst* robot stack:
- Publish simulated joint states + a simple TF for visualization.
- View camera topics (RGB / Depth / Stereo / combined dashboard).
- Send motion goals either as:
  - **JointTrajectory** goals (controller action interface), or
  - **MoveIt 2 MoveGroup** pose goals (a simple “PnP loop”).

> **Important**
> - **End effector is not added yet** (e.g., a **gripper**). Current scripts assume the kinematic chain ends at `link_five`.
> - The “pick & place” loop is **still clunky / not fully perfect** (WIP). It moves between poses, but planning/constraints may fail and there’s no grasping logic yet.
> - Some of the PnP roughness is due to **inverse kinematics limits** (the arm is 5‑DOF, so many full 6D pose goals are over‑constrained).
> - **Velocity & acceleration profiles were heavily edited** compared to the original baseline values.
>   The current known edits are summarized in **Motion profile tuning (velocity & acceleration)** below.

---

## Repository layout

```
Catalyst/
  state_publisher.py         # publishes /joint_states + TF (odom->base_link)
  camera_viewer.py           # OpenCV viewer for ROS Image topics (RGB/Depth/Stereo/combined)
  send_trajectory.py         # FollowJointTrajectory action client (A<->B loop)
  send_trajectory_pnp.py     # MoveIt MoveGroup action client (pose A<->B loop)
```

---

## Prerequisites

You’ll need a working ROS 2 environment with the following packages available:

- `rclpy`, `tf2_ros`, `sensor_msgs`, `geometry_msgs`
- For trajectories: `control_msgs`, `trajectory_msgs`
- For MoveIt loop: `moveit_msgs`, `shape_msgs` (and MoveIt 2 running in the background)
- For camera viewer: `opencv-python` (or system OpenCV), `numpy`, `cv_bridge`

> Tip: if your workspace is ROS 2 Humble / Iron etc., make sure you source it before running anything:
>
> ```bash
> source /opt/ros/<distro>/setup.bash
> ```

---

## Build & run (recommended: `ros2 run`)

### 0) Build the workspace

These scripts are intended to be run as **ROS 2 nodes** using `ros2 run`.

1. Put this repo inside a colcon workspace:

```bash
mkdir -p ~/ws_catalyst/src
# copy the repo here
cd ~/ws_catalyst
colcon build --symlink-install
source install/setup.bash
```

2. Confirm the package is discoverable:

```bash
ros2 pkg list | grep -i catalyst
```

> If this folder is **not yet** an `ament_python` package (no `package.xml` / `setup.py`), convert it (see **Packaging notes** below).


> Replace `<catalyst_pkg>` with your actual ROS 2 package name (commonly `catalyst`).
### 1) State publisher (joint states + TF)

Publishes:
- `joint_states` (`sensor_msgs/JointState`) with:
  `joint_one .. joint_five`
- TF `odom -> base_link` (simple circular motion, useful for visualization)

```bash
ros2 run catalyst state_publisher
```

Use this when you want *something* publishing joint states/TF for RViz/visual debugging.

---

### 2) Camera viewer

`camera_viewer.py` opens an OpenCV window and subscribes to image topics.

Default topic:
- `/cam/rgb/image_raw`

Quit:
- press **q** in the OpenCV window

#### A) View a single RGB topic

```bash
ros2 run catalyst camera_viewer --ros-args -p topic:=/cam/rgb/image_raw
```

#### B) View a depth topic (auto depth visualization)

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p topic:=/cam/depth/image_raw \
  -p depth:=true
```

#### C) Stereo mode (left + right → disparity preview)

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p stereo_mode:=true \
  -p left_topic:=/cam/stereo/left/image_raw \
  -p right_topic:=/cam/stereo/right/image_raw
```

Useful tuning knobs (stereo):
- `stereo_baseline_m` (default `0.06`)
- `stereo_hfov_rad` (default `1.047`)
- `stereo_num_disparities` (must be multiple of 16)
- `stereo_block_size` (odd)

Example with a more “aggressive” matcher:

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p stereo_mode:=true \
  -p stereo_num_disparities:=160 \
  -p stereo_block_size:=9
```

#### D) Combined dashboard (Stereo + RGB + Depth)

Shows a 2×2 dashboard (left, right, rgb, depth):

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p combined_mode:=true \
  -p left_topic:=/cam/stereo/left/image_raw \
  -p right_topic:=/cam/stereo/right/image_raw \
  -p rgb_topic:=/cam/rgb/image_raw \
  -p depth_topic:=/cam/depth/image_raw
```

Other helpful params:
- `window_name` (default `"Camera Feed"`)
- `fps` (GUI refresh rate, default `30.0`)
- `print_encoding_once` (default `true`)

---

### 3) Send joint trajectories (controller action client)

`send_trajectory.py` alternates goals **A ↔ B** on the FollowJointTrajectory action interface.

Defaults:
- `action_name`: `/arm_controller/follow_joint_trajectory`
- `joints`: `["joint_one","joint_two","joint_three","joint_four","joint_five"]`
- `positions_a`: `[0.5, 0.2, -0.3, 0.0, 0.1]`
- `positions_b`: `[0.0, 0.0, 0.0, 0.0, 0.0]`
- `duration_sec`: `0.5`

Run:

```bash
ros2 run catalyst send_trajectory
```

Override positions/duration:

```bash
ros2 run catalyst send_trajectory --ros-args \
  -p duration_sec:=1.5 \
  -p positions_a:="[0.2, 0.1, -0.6, 0.2, 0.0]" \
  -p positions_b:="[0.0, 0.0, 0.0, 0.0, 0.0]"
```

> Note: The action server must already be running (your ros2_control controller).  
> If the action server is not available, the script will fail during startup.

---

### 4) MoveIt “PnP” loop (pose goals A ↔ B)

`send_trajectory_pnp.py` is a **MoveIt 2 MoveGroup action client** that alternates between two EE poses.

Defaults:
- `action_name`: `/move_action`
- `group_name`: `"Catalyst_Manipulator"`
- `base_frame`: `"base_link"`
- `ee_link`: `"link_five"`
- `pose_a_xyz`: `[0.10, -0.42, 0.18]`
- `pose_b_xyz`: `[0.10, -0.34, 0.24]`
- `max_velocity_scaling_factor`: `0.25`
- `max_acceleration_scaling_factor`: `0.25`

Run:

```bash
ros2 run catalyst send_trajectory_pnp
```

Example: slow it down and slightly relax tolerances:

```bash
ros2 run catalyst send_trajectory_pnp --ros-args \
  -p max_velocity_scaling_factor:=0.15 \
  -p max_acceleration_scaling_factor:=0.15 \
  -p position_tolerance:=0.05
```

#### Known limitations (WIP)
- “Pick & Place” here is **only motion between poses** (no grasping).
- Planning can fail depending on scene/constraints. Script optionally retries once with relaxed constraints.
- Because the arm is **5-DOF**, many full 6D pose goals are over‑constrained. If MoveIt planning fails with goal sampling / IK errors, try:
  - using **position-only IK** (or relaxing orientation constraints),
  - moving the goal closer / changing wrist configuration,
  - simplifying the goal pose (position only) and letting orientation float.

- **End effector/gripper is not integrated yet** (URDF/SRDF/controller interfaces still need to be extended).

---

## ROS Interfaces used

### Topics
- `joint_states` — `sensor_msgs/JointState`

### TF
- `odom -> base_link` — broadcast by `state_publisher.py`

### Actions
- `/arm_controller/follow_joint_trajectory` — `control_msgs/action/FollowJointTrajectory`
- `/move_action` — `moveit_msgs/action/MoveGroup`

### Camera topics (defaults)
- RGB: `/cam/rgb/image_raw`
- Depth: `/cam/depth/image_raw`
- Stereo left/right: `/cam/stereo/left/image_raw`, `/cam/stereo/right/image_raw`

---


---

## Packaging notes (only if you don’t already have a ROS 2 package)

`ros2 run` only works if this code is installed as a ROS 2 package.

A minimal approach is to wrap these scripts in an **ament_python** package (example package name: `catalyst`):

1) Create the package skeleton:

```bash
cd ~/ws_catalyst/src
ros2 pkg create --build-type ament_python catalyst --dependencies rclpy sensor_msgs geometry_msgs tf2_ros control_msgs trajectory_msgs moveit_msgs
```

2) Copy the Python files into `catalyst/catalyst/` and keep a `__init__.py`.

3) In `setup.py`, expose executables via `console_scripts`:

```python
entry_points={
    "console_scripts": [
        "state_publisher = catalyst.state_publisher:main",
        "camera_viewer = catalyst.camera_viewer:main",
        "send_trajectory = catalyst.send_trajectory:main",
        "send_trajectory_pnp = catalyst.send_trajectory_pnp:main",
    ],
},
```

4) Rebuild + source again:

```bash
cd ~/ws_catalyst
colcon build --symlink-install
source install/setup.bash
```

After that, all commands in this README will work as `ros2 run catalyst <node>`.


## Launch files

This helper package itself does not ship launch files yet.  
All nodes are currently run as standalone scripts (see commands above).

**Recommended next step:** add a small `launch/` folder and create launch files like:
- `state_publisher.launch.py`
- `camera_viewer.launch.py`
- `trajectory_sender.launch.py`
- `moveit_pnp_loop.launch.py`

(So you can set parameters cleanly and keep your terminal commands short.)

In the broader *Catalyst* workspace, you may also have bringup launch files like:

- `launch/moveit.launch.py` (MoveIt bringup / execution tuning)
- `launch/pnp.launch.py` (Pick & Place test loop bringup)

If those exist in your workspace, prefer launching them via:

```bash
ros2 launch <bringup_package> moveit.launch.py
ros2 launch <bringup_package> pnp.launch.py
```

(Replace `<bringup_package>` with the package that contains the `launch/` directory.)


---

## End effector / gripper TODO

Planned work (not done yet):
- Add an end effector to the URDF (e.g., a gripper link/joint chain).
- Update SRDF / MoveIt group definitions so the EE link is the gripper link (not `link_five`).
- Add a controller/interface for the gripper (ros2_control or a separate action/service).
- Update `send_trajectory_pnp.py` default `ee_link` and (optionally) add grasp/place phases.

---


## Motion profile tuning (velocity & acceleration)

Motion speed in the *Catalyst* stack was **heavily edited** from the original baseline values to make simulation + execution more responsive.

### Summary of edits (from `speed1.txt` / `speed2.txt`)

**A) MoveIt global scaling (planning/execution defaults)**  
File: `Catalyst_moveit_config/config/joint_limits.yaml`

- `default_velocity_scaling_factor`: `0.1 → 0.7 → 1.0`
- `default_acceleration_scaling_factor`: `0.1 → 0.6 → 1.0`

**B) Joint acceleration limits enabled + raised**  
File: `Catalyst_moveit_config/config/joint_limits.yaml`

- `has_acceleration_limits`: `false → true` (all joints)
- Final `max_acceleration`:
  - `joint_one / joint_two / joint_three`: `8.0`
  - `joint_four`: `12.0`
  - `joint_five`: `16.0`

**C) Controller loop updated faster**  
File: `config/Catalyst_controller.yaml`

- `controller_manager.update_rate`: `100 Hz → 500 Hz`

**D) Script-level trajectory duration reduced**  
File: `Catalyst/send_trajectory.py`

- `duration_sec`: `0.5 → 0.3`

**E) Faster tracking under higher speed**  
File: `urdf/Catalyst.urdf.xml`

- Added `position_proportional_gain: 0.6` for `gz_ros2_control` position interfaces (global + per-joint)

**F) Reliability tuning to prevent aborts at higher speed**  
File: `launch/moveit.launch.py`

- `trajectory_execution.allowed_start_tolerance: 0.03`
- plus execution duration monitoring/margins (see your `speed2.txt` notes)

File: `config/Catalyst_controller.yaml`

- relaxed path tolerances / goal timing to reduce `PATH_TOLERANCE_VIOLATED`

### IK / Pick & Place notes (why PnP can be “clunky”)

- **5‑DOF arm vs 6D pose goals:** many pose goals are inherently over‑constrained.  
  Fix: enable **position‑only IK** to improve success rate.  
  File: `Catalyst_moveit_config/config/kinematics.yaml` → `position_only_ik: true`.

- **End-effector SRDF mismatch warning:** MoveIt warning like *“Could not identify parent group for end-effector …”* happened due to an invalid SRDF end-effector definition.  
  Fix: disable/comment the invalid end-effector entry until a real gripper/EE group exists.  
  File: `Catalyst_moveit_config/config/Catalyst_model.srdf`.

> When you add a real gripper/end-effector, revisit SRDF (EE group + parent link near tool), IK settings, and tolerances again.


---

## Notes

- `send_trajectory_pnp.py` includes a clean shutdown guard (`if rclpy.ok(): rclpy.shutdown()`) to avoid Ctrl‑C exceptions during iteration.
- If your camera topics differ, run:
  ```bash
  ros2 topic list | grep -i image
  ```
  and update the viewer parameters.
- If the MoveGroup action name differs in your MoveIt setup, override `action_name`.

---

## License
Add your license here (or keep private/internal).

