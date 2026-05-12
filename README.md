# TB4_Project — Autonomous Trash Collection with TurtleBot 4 + OpenManipulator-X

A ROS 2 Humble project that combines map-based navigation, ArUco-tagged object detection, and a 5-DoF manipulator to drive a TurtleBot 4 to a marker, identify the trash object, and pick-and-place it autonomously.

Developed for **EN.530.707 Robot Systems Programming (RSP)**, Johns Hopkins University, Spring 2026.

---

## Demonstration

### Project overview

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1isiiOjH_H5B2IAAsM0pp6g-UGzWQRGE0/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

### Navigation

Macro-navigation across the lab with Nav2, and closed-loop visual servoing onto an ArUco-tagged target.

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1TDqaOe0okKbLeeknQ6TVHRiTn6oLMGKB/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1sC0yaukU-X11lLwqfCjl5fWIEFh2VKiA/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

### Manipulation

Pick-and-place on the real OpenManipulator-X arm.

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1bfqER796fDf3-FT2vA-Rr2eJQVJ2wEo3/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

---

## Repository Structure

```text
TB4_Project/
├── docs/                            Architecture documentation (start here)
│   ├── index.md
│   ├── navigation.md                Nav2 + visual servoing
│   ├── manipulation.md              OMX + MoveIt2 pick/place
│   └── perception.md                OAK-D + ArUco pipeline
│
├── tb4_openx_sim/                   Gazebo Ignition simulation launch
├── tb4_openx_description/           Combined TB4 + OMX URDF/xacro
├── tb4_openx_navigation/            Nav2 wrappers, maps, mission controllers
├── tb4_openx_manipulation/          Trash-collection action servers
├── tb4_openx_interfaces/            Shared action / message definitions
├── pick_place/                      OMX Pick/Place action server (real-hardware entry point)
└── open_manipulator/                OMX MoveIt config + bringup
```

---

## Documentation

Detailed architecture and implementation notes live in `docs/`:

- **[Navigation](docs/navigation.md)** — differential drive, Nav2 stack, visual servoing, and practical setup notes (parameter overrides, e-stop, CPU constraint).
- **[Manipulation](docs/manipulation.md)** — OMX pick/place, MoveIt2, gripper control.
- **[Perception](docs/perception.md)** — OAK-D camera, ArUco detection pipeline.

---

## Prerequisites

Beyond a standard ROS 2 Humble install, the following packages must be available before `colcon build` will succeed:

```bash
sudo apt install \
  ros-humble-turtlebot4-* \
  ros-humble-nav2-bringup \
  ros-humble-moveit \
  ros-humble-moveit-resources \
  ros-humble-ros-gz \
  ros-humble-tf-transformations
```

The Dynamixel SDK + hardware interface and `ros2_aruco` are vendored under their own subdirectories in this repo and will build together with the rest of the workspace.

---

## Quick Start

Both flows assume the workspace has been built and sourced:

```bash
cd ~/tb4_ws
colcon build
source install/setup.bash
```

### Simulation (laptop only)

```bash
# T1 — Gazebo + controllers + bridges
ros2 launch tb4_openx_sim gazebo_sim.launch.py

# T2 — Nav2
ros2 launch tb4_openx_navigation navigate.launch.py

# T3 — MoveIt2
ros2 launch tb4_openx_manipulation move_group.launch.py

# T4 — Manipulation pipeline (Approach / Pick / Place action servers)
ros2 launch tb4_openx_manipulation manipulation_pipeline.launch.py

# T5 — Mission controller (the full trash-collection task)
ros2 run tb4_openx_navigation mission_controller.py
```

In RViz, click **2D Pose Estimate** to seed AMCL before running Nav2 goals or the mission controller.

### Real robot

The TurtleBot 4 RPi auto-launches `turtlebot4.service` on boot, so the `bringup` step below is only needed if the service is not running. MoveIt2 and the manipulation pipeline run on the laptop rather than the RPi because the RPi 4B does not have enough CPU headroom to run them alongside Nav2 + OAK-D. All commands marked `[RPi]` are run via SSH on the robot; `[laptop]` commands run on your workstation.

```bash
# [RPi] (only if turtlebot4.service is not active)
ros2 launch turtlebot4_bringup standard.launch.py

# [RPi] Clear the Create 3 e-stop — required before any motion command
ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: false}"

# [RPi] Nav2 + map
ros2 launch tb4_openx_navigation real_navigate.launch.py

# [laptop] RViz — click 2D Pose Estimate on the map
ros2 launch turtlebot4_viz view_robot.launch.py

# [laptop] MoveIt2 for the real OMX
ros2 launch tb4_openx_manipulation move_group.launch.py use_sim:=false

# [laptop] Manipulation pipeline
ros2 launch tb4_openx_manipulation real_manipulation_pipeline.launch.py

# [laptop] Run the real-robot mission
ros2 run tb4_openx_navigation real_mission_controller.py
```

---

## Reproducing the Demo: Things That Bite

A few non-obvious gotchas observed on the physical TurtleBot 4 — full details for the navigation items are in [`docs/navigation.md`](docs/navigation.md#practical-setup-notes), and for the manipulation items in [`docs/manipulation.md`](docs/manipulation.md):

- The Create 3 boots with its e-stop engaged; clear it before sending any `/cmd_vel`.
- The default Nav2 `bt_navigator` timing is too tight for the Create 3's odom timestamp behavior. The values that worked for us (in `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`) are documented in the navigation doc.
- The RPi 4B cannot run Nav2 and OAK-D + ArUco detection concurrently — CPU is the bottleneck, not software. The two capabilities are demonstrated sequentially.
- The OMX gripper direction can flip after a 12V power cycle of the arm. Verify gripper motion with a small test command before running an autonomous Pick.
- The pick/place server controls the gripper via a direct `GripperCommand` action client rather than through MoveIt, because MoveIt drops `max_effort` for SRDF named-target gripper trajectories.
