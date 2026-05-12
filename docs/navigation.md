# Navigation Architecture

The navigation pipeline for the TurtleBot 4 OpenX project manages the complete mobility of the robot, from driving across the user environment to making centimeter-level adjustments to pick up an object.

---

## Demonstration

Two short clips show the full pipeline running on real hardware: macro-navigation across the lab using Nav2, and closed-loop visual servoing to stop in front of an ArUco-tagged object.

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1TDqaOe0okKbLeeknQ6TVHRiTn6oLMGKB/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1sC0yaukU-X11lLwqfCjl5fWIEFh2VKiA/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

---

## The TurtleBot 4 Differential Drive
The physical movement of the TurtleBot 4 is based on a **differential drive** system. Instead of steering like a car, the robot relies on two independently powered wheels located on either side of its base.

The robot controls its speed and direction strictly by varying the speed of these two wheels:

* **Driving Straight:** Both wheels spin forward at the exact same speed.
* **Turning:** One wheel spins faster than the other, causing the robot to arc.
* **Spinning in Place:** The wheels spin in opposite directions at the same speed, allowing the robot to rotate on its center axis with a zero turning radius. This is highly useful for scanning warehouse zones for ArUco markers.

---

## Overview
To ensure both speed across long distances and precise alignment for the robotic arm, we split the navigation into two distinct phases:

1. **High-Level (Macro) Navigation:** We use the Nav2 stack to handle driving from the home base to the general vicinity of the target object.

2. **Low-Level (Micro) Navigation:** Once the robot reaches the target zone, standard navigation isn't accurate enough for a robotic arm to grab something. We switch to **Visual Servoing**, which uses the robot's camera to make small, precise wheel adjustments to perfectly align with the target.

---

## High-Level Planning (Macro-Navigation)
Our high-level system relies on two main components to get the robot across the user environment safely:

### Localization (AMCL)
Before the robot can move, it needs to know where it is. We provide the robot with a static 2D occupancy map of the environment (produced beforehand with `slam_toolbox`). Using **Adaptive Monte Carlo Localization (AMCL)**, the robot maintains hundreds of candidate position guesses — called *particles* — and uses each laser scan from the RPLidar to keep particles that match the map well and drop the ones that don't. Over a few seconds the particle cloud converges onto the robot's true pose, which is visible in RViz as the swarm of small arrows that gradually tighten around the robot footprint.

### The Nav2 Stack
Once the robot is localized, the Nav2 stack handles driving. Nav2 is organized around a **Behavior Tree** that orchestrates three kinds of components in sequence:

* **Global Planning:** The `planner_server` (we use the NavFn / A* planner) computes an optimal path from the current pose to the goal pose through the known map, automatically routing around walls and inflated obstacles.
* **Local Control:** The `controller_server` (we use the DWB local planner) tracks that path while continuously scanning for new, unmapped obstacles like a person walking by. It outputs `/cmd_vel` commands that follow the path smoothly without colliding.
* **Recovery:** If progress stalls or the path is blocked, the Behavior Tree falls back to recovery behaviors — clearing the costmap, spinning in place to refresh laser data, or backing up — before retrying the goal.

The default Nav2 configuration for the TurtleBot 4 needs a few parameter overrides to run reliably on real hardware. The most important ones are documented in **Practical Setup Notes** below.

---

## Low-Level Planning (Visual Servoing)
Once Nav2 brings the robot to the correct environment zone, the system transitions to low-level planning to prepare for object retrieval.

**Visual Servoing** is a control method that directly links the robot's camera vision to its motor controls. The pipeline is:

1. The OAK-D camera publishes RGB frames at roughly 30 Hz.
2. `aruco_node` (from the `ros2_aruco` package) detects ArUco markers in each frame and publishes their 3D pose in the camera frame on `/aruco_poses`.
3. `aruco_to_tf` republishes the first detected marker as a TF frame called `marker`, parented to the camera optical frame.
4. The mission controller subscribes to `/aruco_poses` and runs a simple control law: rotate to centre the marker laterally, drive forward or back to reach the target distance, then make small rotation corrections until the marker is square to the camera.

The robot stops once it is within **3 cm laterally**, **2.5 cm of the target distance** (30 cm by default), and **5° of perpendicular**. At that point the OpenManipulator-X arm has enough alignment to reliably grasp the object.

---

## Practical Setup Notes

Running this pipeline on the real TurtleBot 4 surfaces a few issues that don't appear in simulation. They are recorded here for anyone trying to reproduce the demo.

### Nav2 parameter overrides
The default `nav2.yaml` shipped with `turtlebot4_navigation` uses timing parameters that are too aggressive for the iRobot Create 3's odometry-publishing cadence and clock behavior. Without the overrides below, the robot frequently aborts goals after a single spin recovery, or stops short of the target with "transform data too old" errors. The values that worked for us on `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`:

* `bt_navigator.default_server_timeout: 2000` (default `20` is too short — the BT aborts before the planner can acknowledge)
* `bt_navigator.bt_loop_duration: 50` (default `10`)
* `controller_server.FollowPath.transform_tolerance: 30.0` (default `0.2` — the Create 3 publishes `/odom` with timestamp drift that can exceed 900 ms)
* `behavior_server.transform_tolerance: 30.0` (default `0.1`)

### Pre-launch checks on the real robot
The Create 3 boots with its e-stop engaged, so before sending any `/cmd_vel` commands the e-stop must be cleared:
ros2 service call /e_stop irobot_create_msgs/srv/EStop "{e_stop_on: false}"
AMCL also needs an initial pose estimate from RViz (the **2D Pose Estimate** tool) before Nav2 can start — without it the localization lifecycle never reaches `active`.

### CPU constraint: Nav2 and ArUco do not run concurrently
The RPi 4B on board the TurtleBot 4 has four cores and roughly the throughput needed for either the full Nav2 stack **or** the OAK-D + `aruco_node` pipeline, but not both at once. With Nav2 active, the 1-minute CPU load average exceeds 20 (idle near 0%), and `aruco_node` is starved of its share, causing the marker detection rate to drop from a stable 30 Hz to intermittent or zero. The two capabilities are therefore demonstrated sequentially rather than as a single combined run. A platform upgrade to a Raspberry Pi 5 or Jetson Orin Nano would lift this constraint; alternatively, running ArUco detection on the OAK-D's on-board VPU would bypass the host CPU entirely.
