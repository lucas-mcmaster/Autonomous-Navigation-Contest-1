# Autonomous Maze Exploration & SLAM Mapping in ROS 2

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Iron-22314E?logo=ros)
![C++](https://img.shields.io/badge/Language-C%2B%2B17-00599C?logo=c%2B%2B)
![SLAM](https://img.shields.io/badge/Mapping-SLAM%20Toolbox-blue)
![Control](https://img.shields.io/badge/Control-10Hz%20Priority%20FSM-brightgreen)

> **MIE443 Mechatronics Systems: Design & Integration | University of Toronto**  
> *Team Members: Nicolas Rebollo Canedo-Arguelles, Lucas McMaster, Ahmed Fahmi, Bido Mohamed*  
> *Contest 1: "Where am I? Autonomous Robot Exploration of an Unknown Environment"*

---

## Overview

This project implements a fully autonomous exploration, obstacle avoidance, and real-time 2D Simultaneous Localization and Mapping (SLAM) algorithm for the **TurtleBot 4 Lite** in an unknown $4.87 \times 4.87\text{ m}^2$ maze.

Operating under an 8-minute (480 s) fixed-time constraint, the robot navigates without prior map data or human intervention, feeding real-time range data to `slam_toolbox` to construct an accurate 2D occupancy grid of the maze layout and obstacles.

---

##  Control Paradigm & Priority FSM

The controller is implemented as a **6-Priority Hybrid Reactive / Behaviour-Based Finite State Machine (FSM)** running at 10 Hz inside `controlLoop()`:

```text
[ Sensor Inputs: 360° LiDAR, Bumpers, Fused Odom ]
|
v
+-----------------------------------------------+
|       Priority Evaluation Hierarchy (10 Hz)   |
+-----------------------------------------------+
|-- 1a. In-Progress Translation (Distance Tracking)
|-- 1b. In-Progress Rotation (Yaw Feedback)
|-- 2.  Collision Recovery (Bumper Triggered -> Reverse 0.15m)
|-- 3.  Side Obstacle Avoidance (Left/Right < 0.20m -> 15° Pivot)
|-- 4.  Front Obstacle Blocked (Front < 0.30m -> Reorientation)
|-- 5.  Forward Exploration (Clear -> 0.25 m/s | Near Obstacle -> 0.10 m/s)
+-- 6.  Time Limit Exit (>= 480 s -> Safe Stop & Shutdown)
```

---

##  Algorithmic Highlights

### 1. Modified "Seek Open Spaces" with 4:1 Narrow-Corridor Bias
Standard open-space exploration algorithms cause robots to repeatedly traverse wide central areas while ignoring narrow, occluded side corridors. Our algorithm introduces a 4:1 biased selection mechanism:
- **LiDAR Preprocessing & Classification (`classifyRanges`)**: Binarizes 360° range data into arrays of `1` (open) and `0` (blocked) against dynamic thresholds ($0.90 \cdot r_{max}$ for wide space, $0.50 \cdot r_{max}$ for narrow space).
- **Deepest Open Space (`midSequenceFar`)**: Extracts the center of the longest sequence of open space and targets the farthest clear beam.
- **Narrow Passage Extraction (`midSequenceClose`)**: Identifies the shortest open sequence whose geometric arc length exceeds the robot diameter:
  $$\text{Arc Length} = r \cdot \Delta\theta \ge d_{\text{robot}} \quad (0.45\text{ m})$$
- **Bias Modulation**: Every 5th turn (`turn_count_ % 5 == 1`) targets wide open space; the remaining 4 turns prioritize narrow corridors.
- **Angular Constraint**: Reorientations are constrained to $\pm 120^\circ$ to prevent back-and-forth oscillations.

### 2. Spatial Field-of-View (FOV) Filtering (`computeMinFOVDist`)
Segments the raw 360° LiDAR scan into three distinct $40^\circ$ angular windows:
- **Front FOV** ($0^\circ \pm 20^\circ$ with $-90^\circ$ hardware mount offset)
- **Left FOV** ($90^\circ \pm 20^\circ$)
- **Right FOV** ($-90^\circ \pm 20^\circ$)

Applies an inner radius filter ($r \ge 0.15\text{ m}$) to prevent the robot from detecting its own structural standoffs.

### 3. Closed-Loop On-Off Feedback Motion Primitives
- **Angular Control**: Uses continuous odometry yaw feedback with angle normalization:
  $$\text{NormAngle}(\text{yaw} - \text{startYaw}) \in [-\pi, \pi] $$
- **Linear Recovery**: Tracks Euclidean distance from bumper impact coordinates:

  $$\text{Distance} = \sqrt{(x - x_{\text{start}})^2 + (y - y_{\text{start}})^2}$$
  Halts after exactly $0.15\text{ m}$ of reverse travel before initiating directional escape turns.

---

##  Repository File Structure

```text
MIE443-Contest-1/
├── src/
│   └── contest1.cpp          # Complete ROS 2 node, FSM logic, and LiDAR processing
├── maps/
│   ├── Contest1MapPractice.yaml # Practice maze map definitions
│   └── Contest1MapPractice.pgm
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

##  Build & Run Instructions

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select mie443_contest1
source install/setup.bash
```

### Launching SLAM & Exploration Node

1. **Launch TurtleBot 4 Simulation / Bringup:**
   ```bash
   ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py model:=lite
   ```

2. **Launch SLAM Toolbox (Asynchronous Mapping):**
   ```bash
   ros2 launch turtlebot4_navigation slam.launch.py
   ```

3. **Launch Contest 1 Exploration Node:**
   ```bash
   ros2 run mie443_contest1 contest1
   ```

4. **Save Map Upon Completion:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/my_contest1_map
   ```
