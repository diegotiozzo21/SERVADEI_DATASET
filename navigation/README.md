# Autonomous Navigation

This repository provides a complete set of **configuration** and **launch files** to enable **autonomous navigation** for a ROS1-based mobile robot.  
It integrates the **ROS Navigation Stack** for path planning and control with the **LIO-SAM** SLAM algorithm for LiDAR-based localization and mapping.

---

## Overview

The goal of this repository is to make your robot **navigate autonomously** within an unknown environment while simultaneously building a map using **LIO-SAM** and planning trajectories through the **Navigation Stack**.

The repository includes:
- Launch files to start navigation and mapping nodes
- Configuration files for the robot’s sensors, controllers, and planners
- Parameter files for costmaps, and LIO-SAM integration
- Example launch sequences to bring up the system

---

## Dependencies

To use this repository, your system must have the following installed and configured:

### 1. **ROS (Robot Operating System)**
- Tested with **ROS Melodic** on Ubuntu 18.04
- Includes all core navigation packages:
  ```
  sudo apt-get install ros-melodic-navigation
  ```

### 2. **LIO-SAM**
LIO-SAM (LiDAR-Inertial Odometry via Smoothing and Mapping) is required for SLAM-based localization.  
Install it from the official repository:

🔗 **[LIO-SAM GitHub Repository](https://github.com/TixiaoShan/LIO-SAM)**


Ensure the LIO-SAM node runs properly before proceeding.

---

## Key Concepts

- **Localization:** Handled by **LIO-SAM**, which fuses LiDAR, IMU, Wheels Odometry and GNSS data for pose estimation.
- **Mapping:** LIO-SAM continuously refines the map during robot motion.
- **Navigation:** The **ROS Navigation Stack** (move_base) uses this localization for path planning and obstacle avoidance.

---

## Requirements Summary

| Component | Description |
|------------|--------------|
| **ROS Melodic** | Core robotics framework |
| **Navigation Stack** | Motion planning and control |
| **LIO-SAM** | LiDAR-based SLAM |
| **RViz** | Visualization |

---

