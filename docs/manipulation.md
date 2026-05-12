# Manipulation Architecture

While the navigation stack handles getting the robot across the warehouse, the manipulation pipeline is responsible for the actual physical interaction with the environment—specifically, retrieving the target object.

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

## Manipulation Demonstration

Watch the OpenManipulator-X execute the precise pick sequence after the visual servoing alignment is complete:

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1bfqER796fDf3-FT2vA-Rr2eJQVJ2wEo3/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>

<!-- [![Manipulation Demo](assets/manipulation_thumbnail.png)](https://drive.google.com/file/d/1bfqER796fDf3-FT2vA-Rr2eJQVJ2wEo3/view) -->