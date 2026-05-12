# TB4_Project — Autonomous Trash Collection with TurtleBot 4 + OpenManipulator-X

A ROS 2 Humble project that combines map-based navigation, ArUco-tagged object detection, and a 5-DoF manipulator to drive a TurtleBot 4 to a marker, identify the trash object, and pick-and-place it autonomously.

Developed for **EN.530.707 Robot Systems Programming (RSP)**, Johns Hopkins University, Spring 2026.

---

## Demonstration

The full demonstration videos are embedded in [`docs/navigation.md`](docs/navigation.md#demonstration). They show macro-navigation across the lab using Nav2 and closed-loop visual servoing to an ArUco-tagged target.

---

## Repository Structure
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
├── pick_place/                      Legacy pick/place (PickObject/PlaceObject actions)
└── open_manipulator/                OMX MoveIt config + bringup

---

## Documentation

Detailed architecture and implementation notes live in `docs/`:

- **[Navigation](docs/navigation.md)** — differential drive, Nav2 stack, visual servoing, demo videos, and practical setup notes (parameter overrides, e-stop, CPU constraint).
- **[Manipulation](docs/manipulation.md)** — OMX pick/place, MoveIt2, gripper control.
- **[Perception](docs/perception.md)** — OAK-D camera, ArUco detection pipeline.

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

The TurtleBot 4 RPi auto-launches `turtlebot4.service` on boot, so the `bringup` step below is only needed if the service is not running. All commands marked `[RPi]` are run via SSH on the robot; `[laptop]` commands run on your workstation.

```bash
# [RPi] (only if turtlebot4.service is not active)
ros2 launch turtlebot4_bringup standard.launch.py

# [RPi] Clear the Create 3 e-stop — required before any motion command
ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: false}"

# [RPi] Nav2 + map
ros2 launch tb4_openx_navigation real_navigate.launch.py

# [laptop] RViz — click 2D Pose Estimate on the map
ros2 launch turtlebot4_viz view_robot.launch.py

# [RPi] MoveIt2 for the real OMX
ros2 launch tb4_openx_manipulation move_group.launch.py use_sim:=false

# [RPi] Manipulation pipeline
ros2 launch tb4_openx_manipulation real_manipulation_pipeline.launch.py

# [RPi] Run the real-robot mission
ros2 run tb4_openx_navigation real_mission_controller.py
```

---

## Reproducing the Demo: Things That Bite

A few non-obvious gotchas observed on the physical TurtleBot 4 — full details are in [`docs/navigation.md`](docs/navigation.md#practical-setup-notes):

- The Create 3 boots with its e-stop engaged; clear it before sending any `/cmd_vel`.
- The default Nav2 `bt_navigator` timing is too tight for the Create 3's odom timestamp behavior. The values that worked for us (in `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`) are documented in the navigation doc.
- The RPi 4B cannot run Nav2 and OAK-D + ArUco detection concurrently — CPU is the bottleneck, not software. The two capabilities are demonstrated sequentially.
