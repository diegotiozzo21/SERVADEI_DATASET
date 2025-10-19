# GNSS-Based Autonomous Navigation scripts

This repository provides a complete set of **ROS1 Python nodes** for enabling **autonomous navigation using GNSS and odometry data**.  
The workflow performs **online heading calibration**, **GPS waypoint collection**, **coordinate transformation**, and **autonomous navigation**.

---

## Overview

It includes four main scripts:

| Script | Purpose |
|--------|----------|
| `heading_calibration.py` | Computes yaw offset between geographic and magnetic ENU frames |
| `collect_gps_waypoints.py` | Collects GNSS-based waypoints with associated heading |
| `latlon2local.py` | Converts collected waypoints from global (WGS84) to local ENU frame |
| `gps_waypoints_navigation.py` | Sends waypoints to the ROS `move_base` action server for autonomous navigation |

---

## Python Dependencies
```bash
pip install numpy matplotlib pymap3d
```
---

## System Workflow

1. **Heading Calibration (`heading_calibration.py`)**  
   - Outputs a calibrated yaw offset stored in the ROS parameter server (`/calibrated_yaw_offset`).  
   - Generates a plot `online_calibration_results.png` comparing GNSS, SLAM, and GNSS corrected trajectories.

2. **Waypoint Collection (`collect_gps_waypoints.py`)**  
   - Saves each waypoint to `collected_waypoints.txt` when the **spacebar** is pressed.  

3. **Coordinate Transformation (`latlon2local.py`)**  
   - Applies the rotation correction from `/calibrated_yaw_offset`.  
   - Saves transformed waypoints to `trans_points.txt` and creates a visualization `waypoints.png`.

4. **Autonomous Navigation (`gps_waypoints_navigation.py`)**  
   - Reads transformed waypoints from `trans_points.txt`.  
   - Sends sequential navigation goals to the `move_base` action server.  
   - Automatically proceeds to the next waypoint once the previous one is reached.

---

## How to Use

### 1. Prerequisites
Ensure that:
- LIO-SAM and the Navigation Stack are installed and working.
- Topics `/ublox_position_receiver/fix`, `/imu/data`, and `/map_base_link/odometry` are active.

### 2. Run Heading Calibration
```bash
rosrun your_package_name heading_calibration.py
```
Wait until it collects 150 GNSS–odometry pairs.  
A rotation matrix and yaw offset will be computed and stored in `/calibrated_yaw_offset`.

### 3. Collect Waypoints
Run:
```bash
rosrun your_package_name gps_collection.py
```
Move the robot to desired waypoints.  
Press the **spacebar** at each point to record it.  
The collected waypoints are saved in `collected_waypoints.txt`.

### 4. Transform Waypoints to Local Frame
```bash
rosrun your_package_name latlon2local.py
```
This will:
- Convert all waypoints to the local map frame
- Apply yaw correction
- Save them to `trans_points.txt`
- Generate a `waypoints.png` visualization

### 5. Launch Autonomous Navigation
```bash
rosrun your_package_name gps_navigation.py
```
The robot will sequentially navigate to each waypoint using `move_base`.

---


