# Navigation Architecture

The navigation pipeline for this project is split into two distinct phases: **Macro-Navigation** (getting to the general area) and **Micro-Navigation** (precision visual servoing). 

Because the TurtleBot4 is a differential-drive robot, it cannot move laterally. This required a custom state machine to achieve the precision required for robotic arm manipulation.

---

## 1. Macro-Navigation (Nav2)
We use the standard ROS 2 Nav2 stack to handle room-level path planning and obstacle avoidance. 

The `MissionController` node uses an `ActionClient` connected to `/navigate_to_pose`. We defined specific "Zones" in the warehouse map. The robot drives to the center of a zone and stops, intentionally ignoring the exact orientation of the target box.

---

## 2. The Search Maneuver
Once the robot reaches the target zone, it enters an open-loop search state. 
* It spins in place at `0.3 rad/s`.
* It continuously checks the `/aruco_poses` topic.
* If the target is acquired within a 14-second timeout (one full 360° rotation), it immediately applies the brakes and transitions to the alignment phase.

---

## 3. Visual Servoing State Machine
This is the core of the micro-navigation. It loops at 10Hz, reading the camera data and sending velocity commands directly to `/cmd_vel` to square up to the target. 

It executes in a strict priority order:

### State 1: Lateral Centering
The robot rotates in place until the ArUco tag is in the dead-center of the camera frame.
* **Tolerance:** `< 0.03 meters` on the camera's X-axis.

### State 2: Skew Check & Flanking Maneuver
*This is the most critical step for manipulation.* If the robot is looking at the tag diagonally, the arm cannot reach it. We extract the quaternion orientation of the tag to calculate the angle between the camera's Z-axis and the tag's Z-axis.
* **Target:** 180 degrees (Anti-parallel).
* **Tolerance:** `< 10.0 degrees` of skew.
* **Correction:** If the skew is too large, the robot executes a custom **Flanking Maneuver** (detailed below).

### State 3: Distance Approach
Once centered and squared, the robot drives straight forward until it reaches the optimal grasping distance.
* **Target:** Exactly `0.30 meters` (30 cm) from the tag.
* **Tolerance:** `< 0.025 meters`.

---

## 4. The Flanking Maneuver (S-Curve)
Because a differential-drive robot cannot crab-walk, we calculate the lateral distance required to get in front of the box using trigonometry:

`Lateral Distance = Current Distance * sin(Skew Angle)`

The robot then executes a blind 3-step maneuver to change lanes:
1. Turn 90° away from the skew.
2. Drive straight for the calculated `Lateral Distance`.
3. Turn 90° back to face the target.

Once complete, the state machine resets, re-centers the tag, and verifies the skew is resolved before moving in for the final approach.