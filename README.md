# 🤖 ROS2 TurtleBot3 Burger Clone – Built from Scratch in India

![Project Banner](images/banner.jpg)

A fully autonomous ROS2 mobile robot inspired by the official [ROBOTIS TurtleBot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/?utm_source=chatgpt.com), designed, sourced, assembled, and programmed entirely from scratch using locally available components in India.

This project demonstrates that advanced robotics platforms can be built and customized at a fraction of the cost of commercial educational robots while retaining compatibility with the TurtleBot3 software ecosystem.

The robot runs ROS2 Jazzy and implements modern robotics algorithms including SLAM, localization, autonomous navigation, visual perception, QR-code guided autonomous docking, and sensor fusion.

The long-term goal is to create a complete open-source guide that allows students, hobbyists, and engineers to replicate the platform using readily available hardware.

---

# 🎥 Demonstrations

## Autonomous Navigation

[Insert Video]

The robot autonomously navigates to user-defined goals using the ROS2 Navigation Stack (Nav2), performing path planning, obstacle avoidance, and localization in real time.

---

## Autonomous Docking

[Insert Video]

The robot identifies a charging station using QR-code based visual markers, aligns itself with the docking target, performs fine heading corrections, and docks autonomously.

---

## Mapping and Localization

[Insert Video]

Demonstration of SLAM-generated maps and AMCL-based localization using LiDAR and wheel odometry.

---

# 📸 Robot Gallery

## Final Robot Platform

![Robot](images/robot.jpg)

Custom-built TurtleBot3 Burger-inspired mobile robot.

---

## Docking Station

![Dock](images/dock.jpg)

QR-guided docking station used for autonomous charging alignment.

---

## Navigation Visualization

![RViz](images/navigation.png)

RViz visualization showing localization, planning, and navigation.

---

# 🚀 Key Features

## Autonomous Navigation

* ROS2 Navigation2 (Nav2)
* Global path planning
* Local obstacle avoidance
* Costmap-based navigation
* Recovery behaviors
* Goal-based autonomous movement

## Mapping

* 2D SLAM mapping
* Occupancy grid generation
* Persistent map storage
* Real-world environment mapping

## Localization

* Adaptive Monte Carlo Localization (AMCL)
* Particle filter pose estimation
* Sensor fusion with odometry and LiDAR
* Continuous pose correction

## Autonomous Docking

* QR-code based dock detection
* Visual pose estimation
* Heading alignment control
* Close-range approach logic
* Autonomous docking sequence
* Recovery handling when marker visibility is lost

## Perception

* OpenCV-based QR detection
* Camera-based target tracking
* Visual feedback control

## Robot Control

* Differential drive kinematics
* Wheel odometry
* Closed-loop motion control
* IMU-assisted orientation estimation

---

# 🔬 Algorithms and Technologies

### Navigation

* Nav2
* DWB Controller
* Global Planner
* Costmap2D

### Localization

* AMCL
* Particle Filter Localization

### Mapping

* SLAM Toolbox

### Perception

* OpenCV
* QR Pose Estimation

### Motion Estimation

* Wheel Odometry
* IMU Fusion

### Middleware

* ROS2 Jazzy
* DDS Communication

---

# 🛠 Hardware Platform

Unlike the official TurtleBot3 Burger, this robot was assembled entirely from individually sourced components available in India.

### Compute

* Raspberry Pi 5

### Sensors

* 2D LiDAR
* IMU
* Wheel Encoders
* Camera Module

### User Interface

* Onboard Display
* Status Indicators

### Mobility

* Differential Drive Chassis
* DC Gear Motors
* Motor Driver Controller

### Power System

* Battery Pack
* Power Distribution System

---

# 🏗 System Architecture

```text
LiDAR ───────┐
             │
IMU ─────────┼──► Localization (AMCL)
             │
Odometry ────┘

Localization ─► Navigation2 ─► Motion Controller ─► Robot Base

Camera ─► QR Detection ─► Docking Controller ─► cmd_vel
```

---

# 🌟 Technical Highlights

### Custom Autonomous Docking System

Developed a complete visual docking solution that combines QR-code perception, heading estimation, orientation correction, and close-range docking behaviors.

### Continuous Yaw Tracking

Implemented continuous yaw handling to eliminate ±π wraparound discontinuities commonly encountered with IMU-based orientation estimation.

### Real-Time Visual Alignment

Developed dynamic heading correction logic that continuously aligns the robot to the dock while approaching.

### Marker-Loss Recovery

Implemented close-range fallback behavior allowing docking completion even when the QR marker becomes partially or fully invisible near the target.

### Built and Validated on Real Hardware

All algorithms are tested on physical hardware rather than simulation-only environments.

### Cost-Optimized Open Hardware Design

Designed as a low-cost alternative to commercial educational robots while retaining expandability and ROS2 compatibility.

---

# 🛣 Roadmap

## Completed

* [x] ROS2 Jazzy Bringup
* [x] SLAM Mapping
* [x] AMCL Localization
* [x] Navigation2 Integration
* [x] QR Detection
* [x] Autonomous Docking
* [x] Camera Integration
* [x] IMU Integration

## In Progress

* [ ] Docking Reliability Improvements
* [ ] Automatic Charging Verification
* [ ] Enhanced Sensor Fusion

## Future Work

* [ ] Visual SLAM
* [ ] Vision Language Model (VLM) Integration
* [ ] Natural Language Robot Commands
* [ ] Semantic Navigation
* [ ] Object Detection and Tracking
* [ ] Autonomous Mission Execution

---

# 📚 Build Guide (Coming Soon)

A complete build series will be published covering:

1. Hardware Selection
2. Chassis Assembly
3. Electronics Integration
4. Raspberry Pi Setup
5. ROS2 Jazzy Installation
6. LiDAR Configuration
7. Camera Integration
8. Mapping
9. Localization
10. Navigation
11. Autonomous Docking
12. Troubleshooting

The objective is to provide a complete end-to-end guide enabling anyone to recreate the platform from scratch.

---

# 👨‍🔧 Author

Dipanjan Bakshi

Hardware Engineer at Bosch

Robotics Enthusiast | ROS2 Developer | Embedded Systems Builder

Building intelligent autonomous robots one subsystem at a time.
