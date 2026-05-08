# A* Motion Planning for the Sahayak Ground Robot

ROS Melodic implementation of A* pathfinding on 2D LIDAR occupancy grids with Ramer-Douglas-Peucker path simplification, B-spline smoothing, and PID velocity control, developed for the Sahayak ground robot at IvLabs.

## Overview

This package implements a full 2D motion planning pipeline for the Sahayak ground robot. A map is built from 2D LIDAR data using gmapping or hector-slam and stored as a ROS occupancy grid. At runtime, A* searches the grid for a collision-free path from the robot's current pose to a goal, the raw waypoint sequence is simplified using the Ramer-Douglas-Peucker algorithm and smoothed with B-splines, and a PID controller drives the robot's wheel joint velocities along the resulting trajectory. The package integrates with ROS move_base and publishes results to RViz for visualization.

## Methods / Approach

- **Mapping:** gmapping and hector-slam for 2D LIDAR-based occupancy grid construction; maps stored as `.pgm` + `.yaml`
- **Path search:** A* on the occupancy grid; heap-based priority queue (`priority_dict.py`) for open-set management; Euclidean distance heuristic
- **Path simplification:** Ramer-Douglas-Peucker (epsilon=0.7) via the `rdp` library
- **Path smoothing:** B-spline interpolation through simplified waypoints using `scipy.interpolate`
- **Velocity control:** PID controller (K_p=2, K_d=10, K_i=3) for waypoint tracking; publishes to joint velocity controller topics
- **ROS integration:** subscribes to `/map`, `/ground_truth/state`, `/move_base_simple/goal`; publishes to `TrajectoryPlannerROS/global_plan` and per-joint velocity topics
- **Visualization:** RViz path overlay via `rviz_a_star.py`
- **Libraries:** rospy, nav_msgs, geometry_msgs, NumPy, OpenCV (cv2), scipy, rdp, rowan

## Results

Occupancy grid maps of a test environment are included in `src/` (`.pgm` + `.yaml`). A smoothed B-spline path visualization is at `src/curve_smoothing/Figure_1.png`. No quantitative path quality or timing results are recorded in this repository.

## How to Run

This package is not packaged for standalone reproduction. It requires a full ROS Melodic installation and several hardware-specific dependencies tied to the Sahayak robot.

ROS dependencies (install via `apt` or build from source):
```
ros-melodic-navigation  ros-melodic-tf  ros-melodic-ros-control
ros-melodic-cv-bridge   rtabmap-ros     hector-slam
gmapping                laser-scan-matcher
```

Python dependencies:
```
pip install numpy scipy rdp rowan pynput matplotlib
```

Build the package:
```bash
cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch Motion-planning map_launch.launch
```

## Context

Developed at IvLabs, VNIT Nagpur, 2022–2023. Built for the Sahayak ground robot.
