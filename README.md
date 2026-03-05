# Catalyst (ROS 2 + Gazebo Sim + MoveIt 2)

This repository contains a **ROS 2 simulation + MoveIt 2 planning setup** for the **Catalyst** robot, plus a small toolset of Python nodes for:

- running a Gazebo Sim (Fortress/Garden) scene with `gz_ros2_control`
- streaming simple joint trajectories to a `JointTrajectoryController`
- sending MoveIt `MoveGroup` action goals (end-effector pose A/B loop)
- spawning and bridging a **static RGB/Depth/Stereo camera rig** and visualizing it with an OpenCV viewer

> ⚠️ Note: the MoveIt pose-loop demo is functional but still “demo-grade” (tuning/robustness is not finished). See **Known issues & limitations**.

## Demo video

https://github.com/user-attachments/assets/fa072503-8ad9-4ba6-b8ab-829e8f4bf1de
---

## Repo layout

```
Catalyst/                       # ROS 2 package (ament_python)
  package.xml
  setup.py
  setup.cfg
  config/
    Catalyst_controller.yaml
  launch/
    demo.launch.py              # RViz-only: fake joint_state publisher
    gazebo.launch.py            # Gazebo Sim + controllers + optional trajectory streamer + camera spawn/bridge
    moveit.launch.py            # Gazebo Sim + controllers + MoveIt move_group + RViz
    pnp.launch.py               # Gazebo Sim + controllers + MoveIt move_group + EE pose A/B loop sender
  urdf/
    Catalyst.urdf.xml           # robot + gz_ros2_control plugin
    Catalyst.rviz               # RViz config for demo.launch.py
    camera.sdf                  # static camera rig (RGB + depth + stereo)
    camera.urdf.xml             # URDF version of the camera rig (mostly for visualization)
    meshes/
  worlds/
    Catalyst_empty_world.sdf    # includes Sensors system plugin
    Catalyst_green_cube.sdf
  Catalyst/                     # python nodes (rclpy)
    state_publisher.py
    send_trajectory.py
    send_trajectory_pnp.py
    camera_viewer.py

Catalyst_moveit_config/         # MoveIt Setup Assistant output (ament_cmake)
  config/
  launch/
  package.xml
  CMakeLists.txt
```

### Important detail about MoveIt config files
The **Python package `Catalyst` installs the MoveIt config files** from `Catalyst_moveit_config/config/*` into:

```
share/Catalyst/moveit_config/
```

That’s why the custom launch files under `Catalyst/launch/` reference `.../share/Catalyst/moveit_config/...`.

---

## Requirements

This codebase is intended for:

- **ROS 2 Humble**
- **Gazebo Sim** (Fortress / Garden via `ros_gz_sim`)
- **MoveIt 2**
- `ros2_control` + `gz_ros2_control`
- OpenCV + `cv_bridge` (for the camera viewer)

If you already have ROS 2 set up, the most reliable way to pull dependencies is:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> Note: `package.xml` may not list every runtime dependency used by the Python nodes (e.g. `moveit_msgs`, `control_msgs`, `trajectory_msgs`, `geometry_msgs`, `cv_bridge`, etc.). If `rosdep` misses anything, install the missing ROS packages based on the error messages.

---

## Build

1) Create a workspace and put the repo in `src/`:

```bash
mkdir -p ~/catalyst_ws/src
cd ~/catalyst_ws/src
# copy/clone this repo here so you have: ~/catalyst_ws/src/Catalyst/
```

2) Build:

```bash
cd ~/catalyst_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select Catalyst
source install/setup.bash
```

---

## Launch files (what each one does)

### 1) RViz-only demo (no Gazebo)
Publishes a **synthetic `/joint_states`** stream (sinusoidal joints) so you can view the model in RViz without sim.

```bash
ros2 launch Catalyst demo.launch.py
```

What runs:
- `robot_state_publisher` using `urdf/Catalyst.urdf.xml`
- `Catalyst/state_publisher.py` publishing `/joint_states`
- RViz using `urdf/Catalyst.rviz`

---

### 2) Gazebo simulation (empty world) + controllers + optional joint trajectory streamer + camera

```bash
ros2 launch Catalyst gazebo.launch.py
```

What runs:
- Gazebo Sim (`ros_gz_sim`) with `worlds/Catalyst_empty_world.sdf`
- spawns the robot from `/robot_description`
- spawns controllers (see **Controllers** section)
- bridges `/clock` from Gazebo → ROS 2
- spawns a **static camera rig** (from `urdf/camera.sdf`)
- bridges camera topics (RGB + depth + stereo)
- optionally runs `send_trajectory` after a short delay (continuous motion demo)

Useful launch args (current behavior):

```bash
# disable trajectory streamer
ros2 launch Catalyst gazebo.launch.py run_streamer:=false

# change which topic the OpenCV viewer should subscribe to (viewer is usually run separately)
ros2 launch Catalyst gazebo.launch.py camera_topic:=/cam/rgb/image_raw
```

---

### 3) MoveIt + Gazebo simulation (green cube world)

```bash
ros2 launch Catalyst moveit.launch.py
```

What runs:
- Gazebo Sim with `worlds/Catalyst_green_cube.sdf`
- robot spawn + controllers + `/clock` bridge
- MoveIt `move_group`
- RViz using `moveit_config/moveit.rviz`

---

### 4) Pose-loop “pick & place” demo (MoveIt action client)
Despite the name (`pnp.launch.py`), this is currently an **end-effector pose A/B loop** driven by a MoveIt `MoveGroup` action client.

```bash
ros2 launch Catalyst pnp.launch.py
```

What runs:
- Gazebo Sim with `worlds/Catalyst_green_cube.sdf`
- robot spawn + controllers + `/clock` bridge
- MoveIt `move_group`
- after ~6 seconds, runs `ros2 run Catalyst.send_trajectory_pnp` (pose loop)

Tip: this launch does **not** start RViz. You can start it manually:

```bash
RVIZ_CFG=$(ros2 pkg prefix Catalyst)/share/Catalyst/moveit_config/moveit.rviz
rviz2 -d "$RVIZ_CFG"
```

---

## Controllers

Controller configuration file:

- `Catalyst/config/Catalyst_controller.yaml`

Controllers used by launch files:

- `arm_controller` (JointTrajectoryController)
- `arm_broadcaster` (JointStateBroadcaster)

Check controller status:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

---

## Nodes / console scripts

These executables are registered in `setup.py`:

- `state_publisher` → publishes fake `/joint_states` for RViz-only demo
- `send_trajectory` → sends alternating A/B joint goals via `/arm_controller/follow_joint_trajectory`
- `camera_viewer` → OpenCV visualizer for RGB/depth/stereo topics

Run a node directly (after `source install/setup.bash`):

```bash
ros2 run Catalyst send_trajectory
```

> Note: `setup.py` currently declares a `send_trajectory_moveit` console script, but the corresponding Python module is not present in the repo. See **Known issues & limitations**.

---

## Camera system

### Camera model
The camera rig is defined in:

- `urdf/camera.sdf` (used for spawning into Gazebo)
- `urdf/camera.urdf.xml` (URDF version; mostly for visualization)

It contains:

- RGB camera:  `/cam/rgb/image_raw` (1280×720 @ 30 Hz)
- Depth camera: `/cam/depth/image_raw` (640×480 @ 30 Hz)
- Stereo cameras:
  - `/cam/stereo/left/image_raw`
  - `/cam/stereo/right/image_raw`

Gazebo will also publish corresponding `camera_info` topics (bridged in `gazebo.launch.py`):

- `/cam/rgb/camera_info`, `/cam/depth/camera_info`, `/cam/stereo/left/camera_info`, `/cam/stereo/right/camera_info`

### Start the camera (recommended: via gazebo.launch)

```bash
ros2 launch Catalyst gazebo.launch.py
```

This launch **spawns** the camera and starts a `ros_gz_bridge parameter_bridge` for the topics.

### Start the camera viewer (RGB / Depth / Stereo / Combined)

> Gazebo camera streams are typically **BEST_EFFORT QoS**, so make sure your subscribers handle that (the provided viewer does).

**RGB only**

```bash
ros2 run Catalyst camera_viewer --ros-args \
  -p topic:=/cam/rgb/image_raw \
  -p window_name:='Catalyst RGB'
```

**Depth (colormapped)**

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p topic:=/cam/depth/image_raw \
  -p depth:=true \
  -p window_name:='Catalyst Depth'
```

**Stereo depth-from-disparity preview**

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p stereo_mode:=true \
  -p left_topic:=/cam/stereo/left/image_raw \
  -p right_topic:=/cam/stereo/right/image_raw \
  -p window_name:='Catalyst Stereo'
```

**Combined dashboard (Left + Right + RGB + Depth)**

```bash
ros2 run catalyst camera_viewer --ros-args \
  -p combined_mode:=true \
  -p left_topic:=/cam/stereo/left/image_raw \
  -p right_topic:=/cam/stereo/right/image_raw \
  -p rgb_topic:=/cam/rgb/image_raw \
  -p depth_topic:=/cam/depth/image_raw \
  -p window_name:='Catalyst Camera Dashboard'
```

Quit the viewer by focusing the OpenCV window and pressing **`q`**.

### Camera debugging commands

List camera topics:

```bash
ros2 topic list | grep -E '^/cam'
```

Echo camera messages (use BEST_EFFORT):

```bash
ros2 topic echo /cam/rgb/image_raw --qos-reliability best_effort
ros2 topic echo /cam/rgb/camera_info --qos-reliability best_effort
```

If you see topics but no frames:
- confirm the **Sensors** system plugin is present in the world (it is included in `worlds/Catalyst_empty_world.sdf`)
- confirm the `ros_gz_bridge parameter_bridge` is running

---

## Pose-loop parameters (send_trajectory_pnp)

`send_trajectory_pnp.py` drives MoveIt by sending `moveit_msgs/action/MoveGroup` goals. Key parameters:

- `action_name`: MoveGroup action name (default: `/move_action`)
- `group_name`: MoveIt planning group (default is whatever your SRDF defines)
- `base_frame`: planning frame (default: `base_link`)
- `ee_link`: end-effector link name
- `pose_a_xyz`, `pose_b_xyz`: target positions (meters) in `base_frame`
- `pose_a_quat_xyzw`, `pose_b_quat_xyzw`: target orientations
- tolerances: `position_tolerance`, `orientation_tolerance` (+ `constrain_orientation`)
- planning settings: `allowed_planning_time`, `num_planning_attempts`, scaling factors
- loop behavior: `loop_pause_sec`, `retry_on_failure`, `wait_for_server_timeout_sec`

Run it manually with different tuning:

```bash
ros2 run Catalyst.send_trajectory_pnp --ros-args 
  -p pose_a_xyz:='[0.40, 0.00, 0.25]' \  
  -p pose_b_xyz:='[0.30, 0.20, 0.30]' \   
  -p position_tolerance:=0.03 \  
  -p max_velocity_scaling_factor:=0.10 \   
  -p max_acceleration_scaling_factor:=0.10
```

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

- `default_velocity_scaling_factor`: `0.1 → 1.0`
- `default_acceleration_scaling_factor`: `0.1 → 1.0`

**B) Joint acceleration limits enabled + raised**  
File: `Catalyst_moveit_config/config/joint_limits.yaml`

- `has_acceleration_limits`: `false → true` (all joints)
- Final `max_acceleration`:
  - `joint_one / joint_two / joint_three`: `0.0 → 8.0`
  - `joint_four`: `0.0 → 12.0`
  - `joint_five`: `0.0 → 16.0`

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

## Troubleshooting

### Controllers won’t start / action server not available

- Check controller status:
  ```bash
  ros2 control list_controllers
  ```
- Check the action server:
  ```bash
  ros2 action list | grep follow_joint_trajectory
  ```

### No camera frames

- Confirm the camera rig is spawned and topics exist:
  ```bash
  ros2 topic list | grep /cam
  ```
- Use BEST_EFFORT QoS when echoing:
  ```bash
  ros2 topic echo /cam/rgb/image_raw --qos-reliability best_effort
  ```
- Make sure the **Sensors** system plugin is in your world.

### Sim time /clock problems

- Make sure `/clock` is bridged:
  ```bash
  ros2 topic echo /clock
  ```

---

## Next steps (recommended improvements)

- Consider adding a dedicated `camera.launch.py` to re-use the camera rig + bridge in all worlds.
- Make the MoveIt demo more robust (perception-based target acquisition, collision tuning, etc.).
