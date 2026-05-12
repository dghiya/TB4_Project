# Manipulation Architecture

While the navigation stack handles getting the robot across the warehouse, the manipulation pipeline is responsible for the actual physical interaction with the environment—specifically, retrieving the target object.

---

## Manipulation Demonstration

Watch the OpenManipulator-X execute the precise pick sequence after the visual servoing alignment is complete:

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1bfqER796fDf3-FT2vA-Rr2eJQVJ2wEo3/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>

<!-- [![Manipulation Demo](assets/manipulation_thumbnail.png)](https://drive.google.com/file/d/1bfqER796fDf3-FT2vA-Rr2eJQVJ2wEo3/view) -->

---

## The Hardware: OpenManipulator-X

For physical manipulation, we integrated the [OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) by ROBOTIS. 

This is a lightweight, highly capable robotic arm designed specifically for research and education. 

* **4 Degrees of Freedom (DOF):** It has four main joints (base, shoulder, elbow, and wrist), plus a parallel gripper.
* **Smart Actuators:** It is powered by DYNAMIXEL smart servos, which provide high-resolution feedback on their exact joint angles and applied torque.
* **Seamless Integration:** It is fully open-source and officially supported within the ROS 2 ecosystem.

---

## OpenManipulator-X Specifications

| Specification | Value |
|---------------|-------|
| Degrees of Freedom | 4 DOF + Gripper |
| Reach | 380mm |
| Payload | 500g |
| Actuators | DYNAMIXEL XM430-W350 |
| Communication | U2D2 (USB to Dynamixel) |
| Weight | 0.7 kg |
| Repeatability | ±0.2mm |

---

## Autonomous Pick and Place

In this pipeline, the arm acts as the physical worker. Its job is broken down into a highly coordinated sequence that only triggers once the visual servoing system has perfectly aligned the robot base with the target.

### 1. Motion Planning (MoveIt 2)

Before the arm physically moves, it has to "think" about how to do it safely. We use MoveIt 2, the industry standard for robotic motion planning. 

When the mission controller requests a "Pick," MoveIt calculates the precise joint angles required to reach the target object. Crucially, it also checks for collisions, ensuring the arm doesn't accidentally smash into the TurtleBot's camera or the floor while trying to reach the item.

### 2. The Pick Sequence

Once a safe trajectory is found, the arm executes the pick sequence:

* **Extend:** The arm reaches out to the coordinates provided by the perception system.
* **Grasp:** The parallel gripper closes around the target object (identified by its ArUco marker).
* **Tuck for Transit:** Instead of driving with the arm fully extended (which would shift the robot's center of gravity and cause navigation issues), the arm pulls the object back into a compact, safe "Home" position for the drive back.

### 3. The Place Sequence

After the Nav2 stack drives the robot back to the home base, the mission controller triggers the final stage. The arm smoothly extends back out, opens the gripper to drop the retrieved object, and returns to its resting state, ready for the next mission.

---

## Action Interface

The manipulation subsystem exposes two ROS 2 actions defined in the `tb4_openx_interfaces` package. The mission controller (or any external client) invokes these to trigger autonomous pick and place behavior.

### `/pick_object` — `tb4_openx_interfaces/action/Pick`

| Field | Type | Description |
|-------|------|-------------|
| **Goal** `marker_frame` | `string` | TF frame name of the detected target (e.g. `marker`) |
| **Feedback** `current_phase` | `string` | Current execution phase, see table below |
| **Result** `success` | `bool` | True if the full sequence completed |
| **Result** `message` | `string` | Human-readable status string |

### `/place_object` — `tb4_openx_interfaces/action/Place`

| Field | Type | Description |
|-------|------|-------------|
| **Goal** `place_pose` | `geometry_msgs/PoseStamped` | Target drop pose in `world` or `base_link` frame |
| **Goal** `retreat_height` | `float64` | Vertical clearance after release, in meters |
| **Feedback** `current_phase` | `string` | Current execution phase, see table below |
| **Result** `success` | `bool` | True if the full sequence completed |
| **Result** `message` | `string` | Human-readable status string |

The action server is implemented in `pick_place/src/pick_place.cpp`. Both actions provide real-time `current_phase` feedback so the mission controller can log progress and detect partial failures.

---

## Pick and Place Phase Breakdown

Internally, each action is decomposed into a fixed sequence of phases. The action server publishes the current phase as feedback at each transition, which is how the mission controller monitors progress.

### Pick — 6 phases

| # | Phase | Action |
|---|-------|--------|
| 1 | `opening_gripper` | Issue GripperCommand to open position before approach |
| 2 | `approaching` | MoveIt plans + executes to a pre-grasp pose above the marker |
| 3 | `descending` | Linear descent to the grasp pose at the marker's height |
| 4 | `closing_gripper` | Issue GripperCommand to close position around the object |
| 5 | `ascending` | Lift the grasped object back to the pre-grasp height |
| 6 | `returning_home` | Tuck the arm back to the compact home pose for safe driving |

### Place — 5 phases

| # | Phase | Action |
|---|-------|--------|
| 1 | `approaching_place` | MoveIt plans + executes to a pre-place pose above the target |
| 2 | `placing` | Linear descent to the place pose |
| 3 | `opening_gripper` | Issue GripperCommand to release the object |
| 4 | `retreating` | Ascend by `retreat_height` to clear the released object |
| 5 | `returning_home` | Tuck the arm back to the compact home pose |

---

## GripperCommand Bypass Design

A subtle but important design decision in `pick_place.cpp` is that **MoveIt is used for arm motion only — the gripper is controlled by a dedicated `GripperCommand` action client that bypasses MoveIt entirely**.

### Why

MoveIt's `simple_controller_manager` is supposed to honor a `max_effort` parameter from the controllers YAML when commanding the gripper. In practice, it only applies `max_effort` when the trajectory message includes an explicit `effort` field. For SRDF named-target trajectories (e.g. `move_to_named(gripper, "open")`), the trajectory generator does not populate `effort`, so the YAML-configured limit is silently ignored and the gripper drives against its mechanical stop at full torque.

### How

Inside the Pick/Place action server, the gripper phases use a parallel ROS action client connected directly to the low-level `gripper_controller`:

```cpp
auto gripper_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    this, "gripper_controller/gripper_cmd");

// In opening_gripper / closing_gripper phases:
auto goal = control_msgs::action::GripperCommand::Goal();
goal.command.position   = GRIPPER_OPEN_POSITION;   // or GRIPPER_CLOSE_POSITION
goal.command.max_effort = 0.0;                     // honored by hardware controller
gripper_client->async_send_goal(goal);
```

This guarantees the `max_effort = 0.0` limit reaches the Dynamixel firmware, preventing motor jam and damage. MoveIt continues to handle all four arm joints (collision checking, IK, trajectory planning) — only the gripper is detoured.

This split is what made the real-hardware Pick + Place sequence (recorded 5/8) reliably reach `success: true` without manual intervention.

---

## Dual-SRDF Strategy

The OpenManipulator-X gripper joint values are environment-dependent. The repository carries **two SRDF files** loaded by different launch entry points:

| File | Used by | `open` value | `close` value |
|------|---------|--------------|---------------|
| `open_manipulator_x.srdf` | `move_group.launch.py` (sim) | `0.019` | `0.0` |
| `open_manipulator_x_real.srdf` | `move_group.launch.py use_sim:=false` (real) | `-0.030` | `-0.025` |

The "real" values were calibrated empirically against the specific OMX unit used in the project, whose Dynamixel ID 15 motor has a non-standard home offset in EEPROM. With standard values, that motor drives the gripper into a mechanical jam zone around joint position ≈ -0.014.

`hardware_moveit.launch.py` selects the real SRDF via the `use_sim:=false` argument passed to `move_group.launch.py`. The sim launch path uses the standard SRDF without modification, so Gazebo behavior remains consistent with the upstream ROBOTIS configuration.

This dual-SRDF approach keeps the simulation pipeline portable while isolating hardware-specific calibration to a single file, so future users of a differently-calibrated OMX only need to override `open_manipulator_x_real.srdf` rather than touching the rest of the manipulation stack.

