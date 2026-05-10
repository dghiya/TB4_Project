# TB4 OpenX Manipulation Pipeline

Welcome to the official documentation for the **TurtleBot4 Manipulation Pipeline**. This project integrates Nav2, MoveIt 2, and Luxonis OAK-D vision to autonomously locate, align to, and manipulate objects marked with ArUco tags.

## 🚀 Project Overview

The pipeline executes a highly robust state machine:
1. **Macro-Navigation:** Nav2 drives the robot to a designated rough zone.
2. **Visual Search:** The base rotates until the OAK-D camera acquires the target.
3. **Micro-Alignment:** A custom visual servoing controller squares the robot to the target using an S-curve flanking maneuver.
4. **Manipulation:** MoveIt 2 calculates kinematics and executes the pick.

!!! tip "Why this architecture?"
    Separating navigation from visual servoing prevents the `/cmd_vel` topic from getting conflicting commands, resulting in a buttery-smooth handoff to the robotic arm.

## 📚 Where to go next?
* Check out the [Getting Started](tutorials/getting_started.md) guide to install the workspace.
* Read the [Vision Servoing](tutorials/vision.md) tutorial to understand the alignment math.