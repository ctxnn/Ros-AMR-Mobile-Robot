# AMR-FE01 — Autonomous Mobile Robot for Exploration (Version 1)

<p align="center">
  <strong>A ROS 2 + Gazebo Sim differential-drive robot with SLAM, Nav2 autonomous navigation, and computer-vision-based human detection.</strong>
</p>

| Field | Value |
|---|---|
| ROS 2 Distro | Jazzy / Humble |
| Simulator | Gazebo Sim (gz-sim) |
| Build System | `ament_python` |
| Package Name | `amr_fe01` |

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building the Package](#building-the-package)
- [Usage](#usage)
  - [Launch SLAM (Mapping Mode)](#1-launch-slam-mapping-mode)
  - [Launch Nav2 (Autonomous Navigation)](#2-launch-nav2-autonomous-navigation)
  - [Teleoperation](#3-teleoperation)
- [Architecture Deep Dive](#architecture-deep-dive)
  - [Robot Description (URDF / Xacro)](#robot-description-urdf--xacro)
  - [Gazebo Simulation World](#gazebo-simulation-world)
  - [ROS–Gazebo Bridge](#ros-gazebo-bridge)
  - [SLAM Pipeline](#slam-pipeline)
  - [Nav2 Navigation Stack](#nav2-navigation-stack)
  - [Vision Node (Human Detection)](#vision-node-human-detection)
- [TF Tree](#tf-tree)
- [ROS Topic Map](#ros-topic-map)
- [Configuration Reference](#configuration-reference)
- [Known Issues & Troubleshooting](#known-issues--troubleshooting)
- [Demos](#demos)

---

## Overview

**AMR-FE01** is a ROS 2-based project focused on the design, simulation, and implementation of an autonomous mobile robot platform. The robot uses a **differential-drive** locomotion system with two powered wheels and a rear caster, and is equipped with a **360° GPU-accelerated LiDAR** and an **RGB camera** for perception.

The project provides two complete operational modes:

1. **SLAM Mode** — Drive the robot (via teleoperation) to build a 2D occupancy grid map of the environment using the `slam_toolbox` package.
2. **Nav2 Mode** — Use the pre-built map to perform fully autonomous point-to-point navigation with obstacle avoidance, path planning, and collision monitoring.

A key design requirement is **expandability**: the platform provides a stable base that can be customised with additional payloads (robotic arm, additional sensors, etc.) without modifying the core system.

---

## Features

- 🤖 **Custom CAD-based robot model** — Multi-layer chassis with STL meshes from CAD designs, scaled to real-world dimensions
- 🌍 **Rich simulation world** — Indoor room environment with furniture models (desk, chair, coffee table, human figure) from Gazebo Fuel
- 📡 **360° GPU LiDAR** — 360-sample, 10 Hz lidar with 0.1–10 m range and Gaussian noise for realistic scan data
- 📷 **RGB Camera** — 640×480 @ 30 fps camera with 60° horizontal FOV for vision tasks
- 🗺️ **SLAM Toolbox integration** — Online synchronous SLAM with Ceres-based scan matching and loop closure
- 🧭 **Nav2 full stack** — AMCL localisation, NavFn global planner, DWB local planner, collision monitor, and docking server
- 👁️ **Computer vision node** — Real-time human detection using OpenCV's HOG+SVM pedestrian detector with bounding box overlay
- 🔗 **Complete ROS-Gazebo bridge** — 9 bidirectional topic bridges covering clock, odometry, joint states, velocity commands, TF, LiDAR, point clouds, camera, and camera info

---

## Repository Structure

```
amr_fe01/                          # ROS 2 Python package root
│
├── package.xml                    # ROS 2 package manifest (dependencies, build type)
├── setup.py                       # Python package setup (data files, entry points)
├── setup.cfg                      # Script install directory configuration
├── requirements.txt               # Python pip dependencies (numpy)
├── .gitignore
│
├── amr_fe01/                      # Python source module
│   ├── __init__.py
│   └── vision.py                  # Vision node — HOG+SVM human detection
│
├── resource/
│   └── amr_fe01                   # ament resource index marker file
│
├── launch/                        # ROS 2 launch files
│   ├── slam.launch.py             # SLAM mode (mapping) launch
│   └── nav2.launch.py             # Nav2 mode (autonomous navigation) launch
│
├── urdf/
│   └── amr_fe01.xacro             # Robot description (links, joints, sensors, plugins)
│
├── meshes/                        # 3D mesh files for the robot and environment
│   ├── chassis_l1.stl             # Chassis layer 1 (main body)
│   ├── chassis_l2.stl             # Chassis layer 2 (middle platform)
│   ├── chassis_l3.stl             # Chassis layer 3 (top platform)
│   ├── wheel.stl                  # Wheel mesh (used for both left and right)
│   ├── lidar_base_link.stl        # LiDAR housing base
│   ├── lidar_scanner_link.stl     # LiDAR scanner head
│   ├── room.dae                   # Room environment mesh (COLLADA for textures)
│   └── room.stl                   # Room environment mesh (STL for collision)
│
├── world/
│   └── world.sdf                  # Gazebo Sim world definition (SDF 1.10)
│
├── config/                        # Parameter configuration files
│   ├── ros_gz_bridge_gazebo.yaml  # ROS ↔ Gazebo topic bridge mappings
│   ├── nav2_params.yaml           # Nav2 stack parameters (AMCL, planner, controller, collision monitor, docking)
│   ├── async_slam_toolbox_cfg.yaml  # SLAM Toolbox config (async mode)
│   ├── sync_slam_toolbox_cfg.yaml   # SLAM Toolbox config (sync mode)
│   └── slam_toolbox_cfg.yaml       # SLAM Toolbox minimal laser params
│
├── maps/                          # Pre-built map files
│   ├── world_map.pgm              # Occupancy grid image (generated from SLAM)
│   └── world_map.yaml             # Map metadata (resolution, origin, thresholds)
│
├── rviz/
│   └── slam_cfg.rviz              # RViz2 config for SLAM visualisation
│
├── test/                          # ament lint tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
│
└── videos/                        # Demo recordings
    ├── slam_demo.mp4
    └── amr_fe01 nav2 demo.mp4
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 (Humble) or 24.04 (Jazzy) |
| ROS 2 | Humble Hawksbill or Jazzy Jalisco |
| Gazebo Sim | Harmonic (recommended) or Fortress |
| Python | 3.10+ |

### Required ROS 2 Packages

```bash
sudo apt install \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-teleop-twist-keyboard
```

### Python Dependencies

```bash
pip install numpy==1.26.4 opencv-python
```

---

## Installation

```bash
# 1. Create a workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone this repository
git clone https://github.com/ctxnn/Ros-AMR-Mobile-Robot.git

# 3. Install rosdep dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Building the Package

```bash
cd ~/ros2_ws

# Build the package
colcon build --packages-select amr_fe01

# Source the workspace
source install/setup.bash
```

> **Note:** This is an `ament_python` package. The `setup.py` handles installation of all data files (launch files, URDF, meshes, world, config, maps, and rviz configs) into the `share/amr_fe01/` directory. The `setup.cfg` configures the console script install path to `lib/amr_fe01/`.

---

## Usage

### 1. Launch SLAM (Mapping Mode)

Use this mode to **create a map** of a new environment by driving the robot around.

```bash
# Terminal 1 — Launch the full SLAM stack
ros2 launch amr_fe01 slam.launch.py

# Terminal 2 — Teleoperate the robot to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/amr_fe01/cmd_vel
```

**What this launch file starts:**

| Component | Description |
|---|---|
| `GzServer` | Loads `world.sdf` in Gazebo Sim |
| `gz sim -g` | Opens the Gazebo GUI client |
| `robot_state_publisher` | Publishes the URDF to `/robot_description` and joint TFs |
| `gz_spawn_model` | Spawns the robot from `/robot_description` at position (1.0, 1.0, 0.65) |
| `ros_gz_bridge` | Bridges 9 topics between Gazebo and ROS 2 |
| `rviz2` | Opens RViz with the SLAM configuration (`slam_cfg.rviz`) |
| `slam_toolbox` | Runs online synchronous SLAM using `async_slam_toolbox_cfg.yaml` |

**Saving the map** after mapping is complete:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/amr_fe01/maps/world_map
```

---

### 2. Launch Nav2 (Autonomous Navigation)

Use this mode with a **pre-built map** for autonomous point-to-point navigation.

```bash
ros2 launch amr_fe01 nav2.launch.py
```

**What this launch file starts:**

| Component | Description |
|---|---|
| `GzServer` | Loads `world.sdf` in Gazebo Sim |
| `gz sim -g` | Opens the Gazebo GUI client |
| `robot_state_publisher` | Publishes URDF and TFs |
| `gz_spawn_model` | Spawns the robot at (1.0, 1.0, 0.65) |
| `ros_gz_bridge` | 9-topic bridge between Gazebo ↔ ROS 2 |
| `vision_node` | Human detection via camera feed |
| `rviz2` | Opens RViz with Nav2 default view |
| `nav2_bringup` | Full Nav2 stack (AMCL, planner, controller, collision monitor) with the pre-built map |

**To send a navigation goal:**
1. Open RViz2
2. Click **"2D Pose Estimate"** to set the robot's initial pose on the map
3. Click **"Nav2 Goal"** to set a destination — the robot will autonomously plan and navigate

---

### 3. Teleoperation

The robot listens for velocity commands on `/amr_fe01/cmd_vel`. To drive it manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/amr_fe01/cmd_vel
```

Use the keyboard keys (`i`, `j`, `k`, `l`, etc.) to steer the robot.

---

## Architecture Deep Dive

### Robot Description (URDF / Xacro)

**File:** `urdf/amr_fe01.xacro`

The robot is defined as a standard URDF using Xacro macros for material colour properties. The model consists of **6 links** and **5 joints**:

#### Links

| Link | Description | Visual | Collision | Mass |
|---|---|---|---|---|
| `base_link` | Root reference frame (empty) | — | — | — |
| `chassis_link` | Main robot body | 3 STL layers (`chassis_l1/l2/l3.stl`) rendered in gray | Box (160×180×100 mm) + rear caster cylinder | 1.0 kg |
| `left_wheel_link` | Left drive wheel | `wheel.stl` (rotated π rad) in black | Cylinder (r=35 mm, l=20 mm) | 0.2 kg |
| `right_wheel_link` | Right drive wheel | `wheel.stl` in black | Cylinder (r=35 mm, l=20 mm) | 0.2 kg |
| `lidar_link` | LiDAR sensor housing | `lidar_base_link.stl` + cylinder pedestal | Box (60×60×60 mm) | 0.15 kg |
| `camera_link` | RGB camera module | Green box (20×40×20 mm) | — | 0.05 kg |

All STL meshes are exported from CAD at millimetre scale and converted with `scale="0.001 0.001 0.001"` to metres.

#### Joints

| Joint | Type | Parent → Child | Position (xyz) | Axis |
|---|---|---|---|---|
| `base_link_to_chassis_link` | `fixed` | `base_link` → `chassis_link` | (0, 0, 0) | — |
| `camera_link_to_chassis_link` | `fixed` | `chassis_link` → `camera_link` | (0, 0, 0) | — |
| `left_wheel_link_to_chassis_link_joint` | `continuous` | `chassis_link` → `left_wheel_link` | (0.051, 0.1125, 0.034) | Y-axis |
| `right_wheel_link_to_chassis_link_joint` | `continuous` | `chassis_link` → `right_wheel_link` | (0.051, −0.0925, 0.034) | Y-axis |
| `lidar_link_to_chassis_link_joint` | `fixed` | `chassis_link` → `lidar_link` | (0, 0, 0) | — |

#### Gazebo Plugins (defined inside the xacro)

| Plugin | System | Purpose |
|---|---|---|
| `gz-sim-sensors-system` | `gz::sim::systems::Sensors` | Enables sensor simulation (LiDAR, camera) using the OGRE2 render engine |
| `gz-sim-diff-drive-system` | `gz::sim::systems::DiffDrive` | Differential drive controller — listens on `/amr_fe01/cmd_vel`, publishes odometry on `/amr_fe01/odom` and TF on `/tf` |
| `gz-sim-joint-state-publisher-system` | `gz::sim::systems::JointStatePublisher` | Publishes wheel joint positions on `/joint_states` |

**Diff Drive Kinematics:**
- Wheel separation: **0.205 m**
- Wheel radius: **0.035 m**
- Max linear velocity: **±0.5 m/s**
- Max angular velocity: **±1.0 rad/s**
- Max linear acceleration: **±1.0 m/s²**
- Max angular acceleration: **±2.0 rad/s²**

#### Sensors (defined inside the xacro)

**GPU LiDAR** (attached to `lidar_link`):
| Parameter | Value |
|---|---|
| Type | `gpu_lidar` |
| Topic | `/scan` |
| Update rate | 10 Hz |
| Horizontal samples | 360 |
| Angular range | −π to +π (full 360°) |
| Min range | 0.1 m |
| Max range | 10.0 m |
| Range resolution | 0.01 m |
| Noise | Gaussian (mean=0.0, stddev=0.01) |
| Frame ID | `lidar_link` |

**RGB Camera** (attached to `camera_link`):
| Parameter | Value |
|---|---|
| Type | `camera` |
| Topic | `/rgb_camera/image` |
| Update rate | 30 Hz |
| Resolution | 640 × 480 |
| Format | R8G8B8 |
| Horizontal FOV | 1.047 rad (≈ 60°) |
| Clip near / far | 0.05 m / 10 m |

---

### Gazebo Simulation World

**File:** `world/world.sdf`

The simulation world is defined in SDF 1.10 format and creates an indoor office-like environment.

#### World Configuration

| Parameter | Value |
|---|---|
| Physics step size | 1 ms |
| Real-time factor | 1.0 |
| Real-time update rate | 1000 Hz |
| Gravity | (0, 0, −9.8) m/s² |

#### World Plugins

| Plugin | Purpose |
|---|---|
| `gz-sim-physics-system` | Physics simulation engine |
| `gz-sim-user-commands-system` | Handles user interaction commands (move, delete models) |
| `gz-sim-scene-broadcaster-system` | Broadcasts scene state for visualization |
| `gz-sim-contact-system` | Contact/collision detection |

#### Models in the World

| Model | Source | Description |
|---|---|---|
| `ground_plane` | Inline SDF | 100×100 m flat ground with friction |
| `room` | Local mesh (`meshes/room.dae`) | Room walls/structure with collision |
| `Desk` | [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Office%20Desk) | Office desk furniture |
| `casual_female` | [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual%20female) | Human figure for vision testing |
| `CoffeeTable` | [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coffee%20Table) | Coffee table obstacle |
| `DeskChair` | [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Desk%20Chair) | Desk chair obstacle |
| `box` | Inline SDF | 1×1×1 m blue box obstacle |
| `sphere` | Inline SDF | 0.5 m radius red sphere obstacle |
| `sun` | Directional light | Scene illumination with shadows |

> **Note:** Fuel models are referenced via HTTPS URLs and will be **automatically downloaded** by Gazebo Sim on first launch. The room mesh uses a relative path resolved via `GZ_SIM_RESOURCE_PATH` (set automatically by the launch files).

---

### ROS–Gazebo Bridge

**File:** `config/ros_gz_bridge_gazebo.yaml`

The `ros_gz_bridge` translates messages between Gazebo Sim's internal transport and ROS 2 topics. The following 9 bridges are configured:

| ROS 2 Topic | Gazebo Topic | ROS Type | Gz Type | Direction |
|---|---|---|---|---|
| `/clock` | `/clock` | `rosgraph_msgs/Clock` | `gz.msgs.Clock` | GZ → ROS |
| `/odom` | `/amr_fe01/odom` | `nav_msgs/Odometry` | `gz.msgs.Odometry` | GZ → ROS |
| `/joint_states` | `/joint_states` | `sensor_msgs/JointState` | `gz.msgs.Model` | GZ → ROS |
| `/amr_fe01/cmd_vel` | `/amr_fe01/cmd_vel` | `geometry_msgs/Twist` | `gz.msgs.Twist` | ROS → GZ |
| `/tf` | `/tf` | `tf2_msgs/TFMessage` | `gz.msgs.Pose_V` | GZ → ROS |
| `/scan` | `/scan` | `sensor_msgs/LaserScan` | `gz.msgs.LaserScan` | GZ → ROS |
| `/scan/points` | `/scan/points` | `sensor_msgs/PointCloud2` | `gz.msgs.PointCloudPacked` | GZ → ROS |
| `/camera_info` | `/rgb_camera/camera_info` | `sensor_msgs/CameraInfo` | `gz.msgs.CameraInfo` | GZ → ROS |
| `/image` | `/rgb_camera/image` | `sensor_msgs/Image` | `gz.msgs.Image` | GZ → ROS |

Note that `/amr_fe01/cmd_vel` is the **only ROS → GZ** bridge (sending velocity commands into the simulator). All sensor data flows **GZ → ROS**.

---

### SLAM Pipeline

**Launch:** `slam.launch.py`  
**Config:** `config/async_slam_toolbox_cfg.yaml`

The SLAM system uses the **`slam_toolbox`** package with the `online_sync_launch.py` launcher. It consumes LiDAR scans from `/scan` and odometry from `/odom` to produce a 2D occupancy grid map.

#### How it works

```
                ┌──────────┐
  /scan ───────►│          │──────► /map (occupancy grid)
                │  SLAM    │
  /odom ───────►│ Toolbox  │──────► /tf (map → odom transform)
                │          │
                └──────────┘
```

1. The robot's LiDAR produces 360-sample scans at 10 Hz on `/scan`
2. The diff-drive plugin produces odometry on `/odom` (bridged from Gazebo)
3. SLAM Toolbox correlates successive scans using **Ceres Solver** (Levenberg-Marquardt) to estimate the robot's pose
4. It builds an occupancy grid at **0.05 m/pixel** resolution
5. **Loop closure** is enabled — when the robot revisits an area, the graph is optimised to correct drift

#### Key SLAM Parameters

| Parameter | Value | Meaning |
|---|---|---|
| `solver_plugin` | `CeresSolver` | Uses Google's Ceres non-linear least squares solver |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | Robust optimisation strategy |
| `resolution` | 0.05 m/pixel | Map grid cell size |
| `max_laser_range` | 20.0 m | Maximum range for map rastering |
| `minimum_travel_distance` | 0.5 m | Robot must move 0.5 m before a new scan is processed |
| `minimum_travel_heading` | 0.5 rad | Robot must rotate 0.5 rad before a new scan is processed |
| `do_loop_closing` | `true` | Enables graph-based loop closure |
| `loop_search_maximum_distance` | 3.0 m | Search radius for potential loop closures |
| `map_update_interval` | 5.0 s | How often the map is re-published |
| `transform_publish_period` | 0.02 s (50 Hz) | How often the `map → odom` TF is published |

---

### Nav2 Navigation Stack

**Launch:** `nav2.launch.py`  
**Config:** `config/nav2_params.yaml`  
**Map:** `maps/world_map.yaml` + `maps/world_map.pgm`

The navigation stack uses **Nav2** (`nav2_bringup`) for fully autonomous navigation. The pipeline is:

```
  ┌──────────────────── Nav2 Stack ────────────────────┐
  │                                                     │
  │  ┌───────┐    ┌───────────┐    ┌────────────────┐  │
  │  │ AMCL  │───►│  Planner  │───►│  Controller    │  │
  │  │       │    │  (NavFn)  │    │  (DWB Local)   │  │
  │  └───┬───┘    └───────────┘    └───────┬────────┘  │
  │      │                                  │           │
  │  /scan, /map                    cmd_vel_smoothed    │
  │                                         │           │
  │                              ┌──────────▼────────┐  │
  │                              │ Collision Monitor  │  │
  │                              │ (FootprintApproach)│  │
  │                              └──────────┬────────┘  │
  │                                         │           │
  └─────────────────────────────────────────┼───────────┘
                                            │
                                   /amr_fe01/cmd_vel
                                            │
                                     ┌──────▼──────┐
                                     │  Gazebo Sim  │
                                     └─────────────┘
```

#### AMCL (Adaptive Monte Carlo Localisation)

Localises the robot on the pre-built map using particle filter-based pose estimation.

| Parameter | Value |
|---|---|
| `base_frame_id` | `base_link` |
| `odom_frame_id` | `odom` |
| `global_frame_id` | `map` |
| `robot_model_type` | `DifferentialMotionModel` |

#### Global Planner — NavFn

Computes a global path from the robot's current position to the goal using Dijkstra's algorithm on the costmap.

| Parameter | Value |
|---|---|
| Plugin | `nav2_navfn_planner::NavfnPlanner` |
| Algorithm | Grid-based (Dijkstra / A*) |

#### Local Controller — DWB (Dynamic Window Approach)

Generates real-time velocity commands to follow the global path while avoiding dynamic obstacles.

| Parameter | Value |
|---|---|
| Plugin | `dwb_core::DWBLocalPlanner` |
| Max linear velocity | 0.5 m/s |
| Max angular velocity | 1.0 rad/s |
| Linear acceleration limit | ±2.5 m/s² |
| Angular acceleration limit | ±3.2 rad/s² |
| Velocity samples (x / θ) | 20 / 20 |
| Sim time | 1.7 s |
| Goal tolerance (xy / yaw) | 0.25 m / 0.25 rad |
| Critics | `RotateToGoal`, `Oscillation`, `BaseObstacle`, `GoalAlign`, `PathAlign`, `PathDist`, `GoalDist` |

#### Collision Monitor

A safety layer that sits between the controller output and the final velocity command. It monitors laser scan data and slows/stops the robot if obstacles are within the approach polygon.

| Parameter | Value |
|---|---|
| Input topic | `cmd_vel_smoothed` |
| Output topic | `/amr_fe01/cmd_vel` |
| Observation source | `/scan` (laser) |
| Strategy | `FootprintApproach` polygon |
| Time before collision | 2.0 s |

#### Docking Server (Pre-configured)

A docking controller is pre-configured for future use with a charging dock.

| Parameter | Value |
|---|---|
| Plugin | `opennav_docking::SimpleChargingDock` |
| Docking threshold | 0.05 m |
| Staging offset | −0.7 m |

#### Pre-Built Map

The `maps/` directory contains a map generated from SLAM:

| File | Description |
|---|---|
| `world_map.pgm` | Grayscale occupancy grid image |
| `world_map.yaml` | Map metadata |

**Map metadata:**
- **Resolution:** 0.05 m/pixel (5 cm per pixel)
- **Origin:** (−8.452, −5.914, 0) — the bottom-left corner of the map in world coordinates
- **Occupied threshold:** 0.65 (pixels darker than this are considered walls)
- **Free threshold:** 0.196 (pixels lighter than this are considered free space)

---

### Vision Node (Human Detection)

**File:** `amr_fe01/vision.py`  
**Executable:** `vision_node`  
**Active in:** `nav2.launch.py` only

The vision node performs real-time **human detection** using OpenCV's HOG (Histogram of Oriented Gradients) + SVM (Support Vector Machine) pedestrian detector.

#### Pipeline

```
  /image (from camera) ──► CvBridge ──► HOG detectMultiScale ──► Draw bounding boxes ──► /image_classified
```

#### How it works

1. **Subscribes** to `/image` (bridged from Gazebo camera at 30 fps)
2. **Converts** ROS `Image` messages to OpenCV `BGR8` format using `CvBridge`
3. **Runs** `cv2.HOGDescriptor_getDefaultPeopleDetector()` with:
   - Window stride: (8, 8)
   - Padding: (4, 4)
   - Scale: 1.05
4. **Draws** green bounding boxes and confidence labels on detected humans
5. **Overlays** a detection count (`"Humans detected: N"`) at the top-left
6. **Publishes** the annotated image on `/image_classified`
7. **Logs** detections when humans are found

#### Threading Model

The node uses a `SingleThreadedExecutor` running on a dedicated daemon thread. Image subscriptions and publishers each use their own `MutuallyExclusiveCallbackGroup` to prevent callback contention.

---

## TF Tree

```
map
 └── odom                    (published by SLAM Toolbox or AMCL)
      └── base_link          (published by diff_drive plugin via ros_gz_bridge)
           └── chassis_link  (fixed joint — robot_state_publisher)
                ├── left_wheel_link   (continuous joint — robot_state_publisher)
                ├── right_wheel_link  (continuous joint — robot_state_publisher)
                ├── lidar_link        (fixed joint — robot_state_publisher)
                └── camera_link       (fixed joint — robot_state_publisher)
```

- The `odom → base_link` transform comes from the **diff-drive plugin** in Gazebo, bridged to ROS via `/tf`
- The `map → odom` transform comes from **SLAM Toolbox** (mapping mode) or **AMCL** (navigation mode)
- All `chassis_link → *` transforms are published by the **robot_state_publisher** from the URDF

---

## ROS Topic Map

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (via bridge) | All nodes (sim time) |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | gz_spawn_model, RViz |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo (via bridge) | robot_state_publisher |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo + robot_state_publisher | All nodes |
| `/odom` | `nav_msgs/Odometry` | Gazebo (via bridge) | SLAM Toolbox / Nav2 |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (via bridge) | SLAM Toolbox / Nav2 |
| `/scan/points` | `sensor_msgs/PointCloud2` | Gazebo (via bridge) | (available for use) |
| `/amr_fe01/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop | Gazebo (via bridge) |
| `/image` | `sensor_msgs/Image` | Gazebo (via bridge) | vision_node |
| `/camera_info` | `sensor_msgs/CameraInfo` | Gazebo (via bridge) | (available for use) |
| `/image_classified` | `sensor_msgs/Image` | vision_node | (available for visualisation) |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox / map_server | Nav2 / RViz |

---

## Configuration Reference

| Config File | Used By | Purpose |
|---|---|---|
| `ros_gz_bridge_gazebo.yaml` | Both launches | Maps 9 Gazebo ↔ ROS topic pairs |
| `async_slam_toolbox_cfg.yaml` | `slam.launch.py` | Full SLAM Toolbox params (Ceres solver, scan matching, loop closure) |
| `sync_slam_toolbox_cfg.yaml` | (alternate) | Same params for synchronous mode |
| `slam_toolbox_cfg.yaml` | (minimal) | Laser range limits only |
| `nav2_params.yaml` | `nav2.launch.py` | AMCL, planner, controller, collision monitor, docking server |
| `slam_cfg.rviz` | `slam.launch.py` | RViz layout with Grid, Map, and RobotModel displays |
| `world_map.yaml` | `nav2.launch.py` | Map image path, resolution, origin, thresholds |

---

## Known Issues & Troubleshooting

### Fuel models not loading

Gazebo Sim downloads Fuel models on first launch. If you are behind a firewall or have no internet, pre-download them:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Office Desk"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coffee Table"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Desk Chair"
```

### Room mesh not found

The launch files automatically set `GZ_SIM_RESOURCE_PATH` to the package share directory. If you run `gz sim` manually, set it yourself:

```bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix amr_fe01)/share/amr_fe01
```

### Robot spawns incorrectly / falls through ground

The robot spawns at `z=0.65`. If the world has not fully loaded, the robot can fall. Wait for the world to stabilise before interacting.

### SLAM map drifts

Increase `minimum_travel_distance` and `minimum_travel_heading` in the SLAM config for slower, more deliberate mapping. Ensure the robot moves smoothly (avoid jerky teleop).

### Nav2 "transform timeout" errors

These are common during startup while the TF tree is being established. Wait 5–10 seconds after launch for all transforms to become available.

---

## Demos

Demo recordings are available in the `videos/` directory:

- **`slam_demo.mp4`** — Full SLAM mapping session with teleoperation
- **`amr_fe01 nav2 demo.mp4`** — Autonomous Nav2 navigation with goal planning

---

<p align="center">
  Built with ❤️ using ROS 2, Gazebo Sim, Nav2, and SLAM Toolbox
</p>
