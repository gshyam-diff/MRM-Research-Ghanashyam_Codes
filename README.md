# Autonomous Motion Planning for 7-DoF Manipulator
### Research Division | Mars Rover Manipal (MRM)

![ROS](https://img.shields.io/badge/ROS-Kinetic%2FMelodic-orange) ![MoveIt](https://img.shields.io/badge/Framework-MoveIt!-blue) ![Simulation](https://img.shields.io/badge/Sim-Gazebo-green) ![Language](https://img.shields.io/badge/C%2B%2B%2FPython-red)

## 📖 Project Overview
This repository contains the **Motion Planning and Control Stack** developed for a custom **7-Degree-of-Freedom (7-DoF)** robotic manipulator. 

Developed as part of the research division at **Mars Rover Manipal**, this module solves the Inverse Kinematics (IK) and trajectory generation required to perform autonomous **Pick-and-Place** operations.

> **Note:** The URDF models and simulation environments were developed by the mechanical sub-team. This repository hosts the **control logic and motion planning configuration** implemented on top of that hardware interface.

---

## 🦾 Visual Demonstration
*(Simulation of the 7-DoF Arm executing a planned trajectory)*

![7-DoF Pick and Place Demo](https://github.com/gshyam-diff/MRM-Research-Ghanashyam_Codes/blob/main/MRMPickandPlace.gif)
*Figure 1: Gazebo simulation of the custom manipulator executing a Pick-and-Place task using MoveIt!*

---

## 🛠️ Technical Implementation

### 1. Kinematic Configuration (7-DoF)
The custom manipulator features a redundant degree of freedom to optimize reachability in a rover environment.
* **URDF Integration:** Configured the provided URDF/Xacro mechanical models for use within the ROS navigation stack.
* **IK Solver:** Tuned the **KDL (Kinematics and Dynamics Library)** and **TRAC-IK** solvers within MoveIt to handle the redundancy and optimize for custom joint limits.

### 2. Motion Planning (MoveIt!)
The system utilizes the **MoveIt! Motion Planning Framework** to generate collision-free trajectories.
* **OMPL:** Utilized the Open Motion Planning Library (RRT* and PRM planners) to compute paths in high-dimensional configuration space.
* **Planning Groups:** Configured separate planning groups for the `arm` (7 joints) and the `gripper` (end-effector) to allow for decoupled control.

### 3. Simulation & Configuration
* **System Configuration:** Utilized the **MoveIt Setup Assistant** to generate the Semantic Robot Description (SRDF), collision matrix, and controller launch files, ensuring rapid integration with the simulation environment.
* **Gazebo Testing:** Leveraged the team's existing Gazebo physics environment to verify obstacle avoidance and reachability before real-world trials.
* **Execution Validation:** Validated the **Python Control Scripts** by executing full "Pick-and-Place" sequences in simulation, verifying that the inverse kinematics solutions were executable by the hardware drivers.
