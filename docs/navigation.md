# Navigation Architecture

The navigation pipeline for the TurtleBot 4 OpenX project manages the complete mobility of the robot, from driving across the user environment to making millimeter-level adjustments to pick up an object.

---

## The TurtleBot 4 Differential Drive
The physical movement of the TurtleBot 4 is based on a **differential drive** system. Instead of steering like a car, the robot relies on two independently powered wheels located on either side of its base. 

The robot controls its speed and direction strictly by varying the speed of these two wheels:

* **Driving Straight:** Both wheels spin forward at the exact same speed.
* **Turning:** One wheel spins faster than the other, causing the robot to arc.
* **Spinning in Place:** The wheels spin in opposite directions at the same speed, allowing the robot to rotate on its center axis with a zero turning radius. This is highly useful for scanning warehouse zones.

---

## Overview
To ensure both speed across long distances and extreme accuracy for the robotic arm, we split the navigation into two distinct phases:

1. **High-Level (Macro) Navigation:** We use the Nav2 stack to handle driving from the home base to the general vicinity of the target object. 

2. **Low-Level (Micro) Navigation:** Once the robot reaches the target zone, standard navigation isn't accurate enough for a robotic arm to grab something. We switch to **Visual Servoing**, which uses the robot's camera to make tiny, precise wheel adjustments to perfectly align with the target.

---

## High-Level Planning (Macro-Navigation)
Our high-level system relies on two main concepts to get the robot across the user environment safely:

### Localization (AMCL)
Before the robot can move, it needs to know where it is. We provide the robot with a static 2D map of the environment. Using a technique called Adaptive Monte Carlo Localization (AMCL), the robot uses its laser scanner to look at the physical walls and obstacles around it, comparing what it sees to the map to constantly calculate its exact position and orientation.

### The Nav2 Stack
Once the robot knows where it is, it uses the Nav2 software stack to drive. 

* **Global Planning:** The robot calculates the most efficient, shortest path from its current location to the target zone, drawing a line through the known map.
* **Local Control:** As the robot follows that line, its sensors constantly watch for new, unmapped obstacles (like a person walking by). It dynamically adjusts the wheel speeds to smoothly steer around these hazards without losing its way.
* **Recovery:** If the robot gets trapped or its path is completely blocked, it uses a behavior tree to execute recovery maneuvers, such as backing up or spinning in place to clear its sensors and find a new route.

---

## Low-Level Planning (Visual Servoing)
Once Nav2 brings the robot to the correct environment zone, the system transitions to low-level planning to prepare for object retrieval. 

**Visual Servoing** is a control method that directly links the robot's camera vision to its motor controls. Instead of driving blindly to a GPS coordinate, the robot uses its camera to actively look for the target marker (the ArUco tag).

Once the marker is spotted, the visual servoing algorithm calculates exactly how far away the object is and how skewed the angle is. It then sends continuous, micro-adjustments to the differential drive wheels. The robot carefully stagger-steps—translating and rotating—until the marker is perfectly centered in the camera's view at the exact distance required for the OpenManipulator-X arm to reach it. 

---

## Navigation Demonstration


<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1TDqaOe0okKbLeeknQ6TVHRiTn6oLMGKB/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>

<div style="text-align: center;">
  <iframe src="https://drive.google.com/file/d/1sC0yaukU-X11lLwqfCjl5fWIEFh2VKiA/preview" width="640" height="480" allow="autoplay" allowfullscreen></iframe>
</div>
<br>
