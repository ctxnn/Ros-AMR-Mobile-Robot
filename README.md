# 🛒 Autonomous Supermarket Assistant Robot — ROS 2 Simulation

<p align="center">
  <strong>Design and simulation of a differential-drive autonomous mobile robot for large supermarkets — assists customers in locating products via SLAM-based mapping, Nav2 autonomous navigation, and computer-vision-based customer detection.</strong>
</p>

| Field | Value |
|---|---|
| ROS 2 Distro | Jazzy / Humble |
| Simulator | Gazebo Sim (gz-sim) |
| Build System | `ament_python` |
| Package Name | `supermarketbot` |
| License | MIT |

---

## Table of Contents

- [Project Overview](#1-project-overview)
- [Features](#2-features)
- [Robot Design Parameters](#3-robot-design-parameters)
- [Kinematics](#4-differential-drive-kinematics)
- [3D Modeling & URDF](#5-3d-modeling--urdf)
- [Gazebo Simulation World](#6-gazebo-simulation-supermarket-world)
- [ROS 2 Architecture](#7-ros-2-architecture)
- [SLAM — Mapping the Supermarket](#8-slam--mapping-the-supermarket)
- [Nav2 — Autonomous Navigation](#9-nav2--autonomous-product-navigation)
- [Computer Vision — Customer Detection](#10-computer-vision--customer-detection)
- [Getting Started](#11-getting-started)
- [Repository Structure](#12-repository-structure)
- [Troubleshooting](#13-troubleshooting)

---

## 1. Project Overview

The objective of this project is to design and simulate an **autonomous mobile robot** for large supermarket environments. The robot assists customers by autonomously navigating to product locations within the store.

The project follows a structured robotics simulation workflow:

1. **Parameter Selection** — Defining physical and kinematic constraints for a supermarket environment
2. **3D Modeling & URDF** — Creating the robot's digital twin in Fusion 360, exporting to URDF via Xacro
3. **Simulation (Gazebo / RViz)** — Testing in a virtual supermarket environment with shelves, aisles, and obstacles
4. **Control & Navigation** — Implementing teleoperation, SLAM-based mapping, and Nav2 autonomous path planning

### How Product Location Works

The supermarket is divided into **zones** (aisles), each mapped to a product category:

| Zone | Aisle Position | Product Category |
|---|---|---|
| Zone A | Shelf Row 1 (y ≈ 3.0) | Beverages |
| Zone B | Shelf Row 2 (y ≈ 0.5) | Snacks |
| Zone C | Shelf Row 3 (y ≈ −2.0) | Dairy |
| Zone D | Shelf Row 4 (y ≈ −4.5) | Household |

When a customer requests a product, the system:
1. Looks up the product's zone in a coordinate database
2. Sends the target `(x, y)` as a **Nav2 goal**
3. The robot autonomously plans a path through the aisles, avoids obstacles and customers, and arrives at the correct shelf
4. The onboard camera and vision node ensure safe navigation around people

---

## 2. Features

- 🤖 **Custom CAD-designed robot** — Multi-layer chassis modeled in Fusion 360, exported as STL meshes
- 🏪 **Supermarket simulation world** — Indoor environment with shelving aisles, checkout counter, product crates, and a customer model
- 📡 **360° GPU LiDAR** — 360-sample, 10 Hz, 0.1–10 m range with Gaussian noise for realistic aisle mapping
- 📷 **RGB Camera** — 640×480 @ 30 fps for customer detection
- 🗺️ **SLAM Toolbox** — Online synchronous SLAM with Ceres-based scan matching and loop closure for map building
- 🧭 **Nav2 Full Stack** — AMCL localisation, NavFn global planner, DWB local controller, collision monitor
- 👁️ **Customer Detection** — Real-time HOG+SVM pedestrian detector for safe navigation around shoppers
- 🔗 **ROS–Gazebo Bridge** — 9 bidirectional topic bridges for seamless sim-to-ROS communication

---

## 3. Robot Design Parameters

For a supermarket environment, the robot must balance **maneuverability** (narrow aisles ~1.5–2.0 m), **stability** (smooth indoor floors), and **human-safe interaction** (matching walking speed).

| Parameter | Selected Value | Justification |
|---|---|---|
| **Configuration** | Differential Drive | Simplest kinematics, highly maneuverable in narrow aisles, well-supported by ROS `diff_drive_controller` |
| **Wheel Separation** | 0.205 m | Compact enough for standard supermarket aisles while maintaining turning stability |
| **Wheel Radius** | 0.035 m | Optimised for smooth indoor floors; ensures a low center of gravity |
| **Max Linear Velocity** | 0.5 m/s | Matches comfortable human walking speed for safe assistance |
| **Max Angular Velocity** | 1.0 rad/s | Allows responsive turning in tight aisle intersections |
| **Payload** | Sensors (LiDAR, Camera) + display | Sufficient for perception stack; chassis designed for sensor mounting |
| **Caster Wheel** | Rear passive caster | Provides 3-point stability without adding kinematic complexity |

---

## 4. Differential Drive Kinematics

Based on *Introduction to Autonomous Mobile Robots* (Siegwart et al., Chapter 3), the robot uses **differential drive kinematics** where two independently driven wheels control forward motion and rotation.

### Kinematic Model

The robot's pose in the world frame is defined by $(x, y, \theta)$.

Given wheel velocities $v_L$ (left) and $v_R$ (right), the robot's **linear velocity** $v$ and **angular velocity** $\omega$ are:

$$v = \frac{r}{2}(v_R + v_L)$$

$$\omega = \frac{r}{L}(v_R - v_L)$$

Where:
- $r = 0.035$ m — wheel radius
- $L = 0.205$ m — wheel separation (track width)

### Motion Primitives

| Motion | Condition | Result |
|---|---|---|
| **Straight** | $v_L = v_R$ | Pure forward/backward, $\omega = 0$ |
| **Pivot Turn** | $v_L = -v_R$ | Rotation in place, $v = 0$ |
| **Arc Turn** | $v_L \neq v_R$ | Curved path with radius $R = \frac{L}{2} \cdot \frac{v_R + v_L}{v_R - v_L}$ |

### Why Differential Drive?

| | Differential Drive | Mecanum | Ackermann |
|---|---|---|---|
| Aisles | ✅ Zero-turn in tight spaces | ✅ Strafing possible | ❌ Wide turning radius |
| Complexity | ✅ 2 motors | ❌ 4 motors + complex | ❌ Steering linkage |
| ROS Support | ✅ `diff_drive_controller` | ⚠️ Custom controller | ⚠️ Limited |
| Indoor Floors | ✅ Ideal | ✅ Works | ✅ Works |

---

## 5. 3D Modeling & URDF

### CAD to URDF Pipeline

```
Fusion 360 → Export STL (mm scale) → Xacro (scale 0.001) → robot_state_publisher → Gazebo Sim
```

1. The robot chassis was modeled in **Fusion 360** as three stacked layers
2. Each component was exported as an **STL mesh** at millimetre scale
3. A **Xacro** file defines the URDF with `scale="0.001 0.001 0.001"` to convert mm → metres
4. `base_link` is placed at the geometric center of the drive axle

### Link Tree

```
base_link (reference frame)
 └── chassis_link (fixed)
      ├── left_wheel_link (continuous, Y-axis)
      ├── right_wheel_link (continuous, Y-axis)
      ├── lidar_link (fixed)
      └── camera_link (fixed)
```

### Links

| Link | Mesh / Geometry | Mass | Purpose |
|---|---|---|---|
| `base_link` | — (reference frame) | — | Root TF frame |
| `chassis_link` | 3 STL layers (l1, l2, l3) | 1.0 kg | Main robot body |
| `left_wheel_link` | `wheel.stl` | 0.2 kg | Left drive wheel |
| `right_wheel_link` | `wheel.stl` | 0.2 kg | Right drive wheel |
| `lidar_link` | `lidar_base_link.stl` + cylinder | 0.15 kg | LiDAR sensor housing |
| `camera_link` | Green box (20×40×20 mm) | 0.05 kg | RGB camera module |

### Joints

| Joint | Type | Parent → Child | Axis |
|---|---|---|---|
| `base_link_to_chassis_link` | `fixed` | base → chassis | — |
| `left_wheel_..._joint` | `continuous` | chassis → left_wheel | Y |
| `right_wheel_..._joint` | `continuous` | chassis → right_wheel | Y |
| `lidar_..._joint` | `fixed` | chassis → lidar | — |
| `camera_..._joint` | `fixed` | chassis → camera | — |

### Sensors

**GPU LiDAR** — For aisle mapping and obstacle detection:

| Parameter | Value |
|---|---|
| Samples | 360 (1° resolution) |
| Range | 0.1 – 10.0 m |
| Update Rate | 10 Hz |
| Noise | Gaussian (σ = 0.01) |
| Topic | `/scan` |

**RGB Camera** — For customer detection:

| Parameter | Value |
|---|---|
| Resolution | 640 × 480 |
| FOV | 60° horizontal |
| Update Rate | 30 Hz |
| Topic | `/rgb_camera/image` |

### Gazebo Plugins (embedded in Xacro)

| Plugin | Purpose |
|---|---|
| `gz::sim::systems::DiffDrive` | Differential drive controller — subscribes to `/supermarketbot/cmd_vel`, publishes odometry |
| `gz::sim::systems::Sensors` | Enables LiDAR + camera simulation via OGRE2 renderer |
| `gz::sim::systems::JointStatePublisher` | Publishes wheel joint states for TF |

---

## 6. Gazebo Simulation — Supermarket World

The simulation world (`world/world.sdf`) recreates an indoor supermarket environment using SDF 1.10.

### World Layout (16 m × 12 m)

```
  ┌─────────────────── 16 m ───────────────────┐
  │                                              │
  │  [Checkout]                                  │  
  │                                              │
  │  ┌─shelf_1a─┐  gap  ┌─shelf_1b─┐           │  Aisle 1 (Beverages)
  │  └──────────┘       └──────────┘           │
  │                                              │
  │  ┌─shelf_2a─┐  gap  ┌─shelf_2b─┐           │  Aisle 2 (Snacks)
  │  └──────────┘   🤖  └──────────┘           │  ← Robot spawns here
  │                                              │
  │  ┌─shelf_3a─┐  gap  ┌─shelf_3b─┐   👤      │  Aisle 3 (Dairy)
  │  └──────────┘       └──────────┘           │
  │                                              │
  │  ┌─shelf_4a─┐  gap  ┌─shelf_4b─┐           │  Aisle 4 (Household)
  │  └──────────┘       └──────────┘           │
  │                                              │
  └──────────────────────────────────────────────┘
```

### Models

| Model | Type | Description |
|---|---|---|
| 4 walls | SDF box primitives | 16×12 m enclosure, 2.5 m height |
| 8 shelf units | SDF box primitives | 5.0×0.5×1.6 m each, brown wood color |
| Checkout counter | SDF box primitive | 2.0×1.0×1.0 m, near entrance |
| Product crates (×2) | SDF box primitives | Small obstacles in aisles |
| Customer | [Gazebo Fuel](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual%20female) | Human model for vision node testing |

Each shelf row has a **2 m cross-aisle gap** in the middle (at x ≈ 0) connecting all aisles, allowing the robot to move between them efficiently.

### Why This Layout Works for SLAM

- **Clear geometric features** — Wall edges and shelf surfaces provide strong LiDAR returns
- **Non-symmetric layout** — Checkout counter and crate positions break symmetry, helping loop closure
- **Appropriate scale** — Aisles are 1.5–2.5 m wide, matching real supermarket dimensions
- **LiDAR visibility** — Shelves are 1.6 m tall; the LiDAR at ~0.8 m height sees shelf walls clearly in the 2D scan

---

## 7. ROS 2 Architecture

### TF Tree

```
map
 └── odom                    (SLAM Toolbox or AMCL)
      └── base_link          (diff_drive plugin → ros_gz_bridge)
           └── chassis_link  (robot_state_publisher)
                ├── left_wheel_link   (continuous joint)
                ├── right_wheel_link  (continuous joint)
                ├── lidar_link        (fixed)
                └── camera_link       (fixed)
```

### ROS–Gazebo Bridge

9 topic bridges configured in `config/ros_gz_bridge_gazebo.yaml`:

| ROS 2 Topic | Direction | Purpose |
|---|---|---|
| `/clock` | GZ → ROS | Simulation time sync |
| `/odom` | GZ → ROS | Odometry from diff-drive |
| `/joint_states` | GZ → ROS | Wheel joint positions |
| `/supermarketbot/cmd_vel` | ROS → GZ | Velocity commands to robot |
| `/tf` | GZ → ROS | Transform tree updates |
| `/scan` | GZ → ROS | LiDAR laser scans |
| `/scan/points` | GZ → ROS | LiDAR point cloud |
| `/camera_info` | GZ → ROS | Camera intrinsic parameters |
| `/image` | GZ → ROS | RGB camera feed |

### Key Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/supermarketbot/cmd_vel` | `Twist` | Nav2 / teleop | Gazebo (via bridge) |
| `/scan` | `LaserScan` | Gazebo | SLAM / Nav2 |
| `/odom` | `Odometry` | Gazebo | SLAM / Nav2 |
| `/image` | `Image` | Gazebo | vision_node |
| `/image_detected` | `Image` | vision_node | RViz (visualisation) |
| `/map` | `OccupancyGrid` | SLAM / map_server | Nav2 / RViz |

---

## 8. SLAM — Mapping the Supermarket

**Config:** `config/async_slam_toolbox_cfg.yaml`

The robot uses SLAM Toolbox to build a 2D occupancy grid map of the supermarket by driving through the aisles.

### Pipeline

```
  /scan (LiDAR) ──┐
                    ├──► SLAM Toolbox ──► /map (occupancy grid)
  /odom ───────────┘         │
                             └──► /tf (map → odom)
```

### How It Works

1. The robot's LiDAR produces **360 samples** per scan at 10 Hz
2. SLAM Toolbox correlates successive scans using the **Ceres Solver** (Levenberg-Marquardt trust strategy)
3. It builds an occupancy grid at **0.05 m/pixel** resolution
4. **Loop closure** is enabled — when the robot returns to a previously visited aisle, the map graph is optimised to correct accumulated drift

### Key Parameters

| Parameter | Value | Effect |
|---|---|---|
| `solver_plugin` | `CeresSolver` | Non-linear least squares scan matching |
| `resolution` | 0.05 m/pixel | 5 cm grid cells — sufficient for aisle-level detail |
| `minimum_travel_distance` | 0.5 m | New scan processed every 0.5 m of movement |
| `do_loop_closing` | `true` | Corrects drift when revisiting aisles |
| `loop_search_maximum_distance` | 3.0 m | Searches 3 m radius for loop closure matches |
| `max_laser_range` | 20.0 m | Maximum range for map rastering |

### Generating the Map

```bash
# Terminal 1 — Launch SLAM
ros2 launch supermarketbot slam.launch.py

# Terminal 2 — Drive through all aisles
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

# Terminal 3 — Save the map when complete
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/supermarketbot/maps/world_map
```

> **Note:** After building the supermarket world, you must generate a new map before using Nav2 navigation mode. The included `maps/world_map.pgm` is a placeholder and should be regenerated.

---

## 9. Nav2 — Autonomous Product Navigation

**Config:** `config/nav2_params.yaml`

The full Nav2 stack handles autonomous navigation from the robot's current position to a target product shelf.

### Architecture

```
  ┌──────────────────── Nav2 Stack ─────────────────────┐
  │                                                      │
  │  ┌───────┐    ┌──────────┐    ┌──────────────────┐  │
  │  │ AMCL  │───►│ NavFn    │───►│ DWB Local        │  │
  │  │       │    │ Planner  │    │ Controller       │  │
  │  └───┬───┘    └──────────┘    └────────┬─────────┘  │
  │      │                                  │            │
  │  /scan, /map                    cmd_vel_smoothed     │
  │                                         │            │
  │                              ┌──────────▼─────────┐  │
  │                              │ Collision Monitor   │  │
  │                              │ (safe around        │  │
  │                              │  customers)         │  │
  │                              └──────────┬─────────┘  │
  └──────────────────────────────────────────┼───────────┘
                                             │
                                   /supermarketbot/cmd_vel
                                             │
                                      ┌──────▼──────┐
                                      │  Gazebo Sim  │
                                      └─────────────┘
```

### Components

**AMCL** — Localises the robot on the pre-built supermarket map using a particle filter.

**NavFn Planner** — Computes a global path from current position to the target shelf using Dijkstra's algorithm on the costmap.

**DWB Controller** — Generates real-time velocity commands to follow the plan while avoiding dynamic obstacles (customers, crates).

| Parameter | Value |
|---|---|
| Max linear velocity | 0.5 m/s |
| Max angular velocity | 1.0 rad/s |
| Goal tolerance (xy) | 0.25 m |
| Goal tolerance (yaw) | 0.25 rad |
| Critics | RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist |

**Collision Monitor** — Safety layer between the controller and the robot. Monitors `/scan` data and slows/stops the robot if obstacles (e.g., a customer stepping into the aisle) are within the approach polygon.

### Sending a Navigation Goal

1. Launch: `ros2 launch supermarketbot nav2.launch.py`
2. In RViz, click **"2D Pose Estimate"** to set the robot's initial position
3. Click **"Nav2 Goal"** to set the target shelf location
4. The robot autonomously plans through the aisles and navigates to the product

---

## 10. Computer Vision — Customer Detection

**File:** `supermarketbot/vision.py`

The vision node ensures **safe navigation around customers** by detecting pedestrians in the camera feed.

### Pipeline

```
  /image (camera) ──► CvBridge ──► HOG detectMultiScale ──► Bounding boxes ──► /image_detected
```

### How It Works

1. **Subscribes** to `/image` (RGB camera at 30 fps)
2. **Converts** ROS Image → OpenCV BGR8 via `CvBridge`
3. **Runs** `cv2.HOGDescriptor_getDefaultPeopleDetector()` — a pre-trained Histogram of Oriented Gradients + SVM pedestrian detector
4. **Draws** green bounding boxes with confidence labels on detected customers
5. **Publishes** annotated image on `/image_detected`
6. **Logs** detections for monitoring

### Detection Parameters

| Parameter | Value |
|---|---|
| Window stride | (8, 8) |
| Padding | (4, 4) |
| Scale | 1.05 |
| Output | Bounding boxes + confidence scores |

---

## 11. Getting Started

### Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 or 24.04 |
| ROS 2 | Humble or Jazzy |
| Gazebo Sim | Harmonic (recommended) |
| Python | 3.10+ |

### Install Dependencies

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

pip install numpy==1.26.4 opencv-python
```

### Clone & Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/ctxnn/Ros-AMR-Mobile-Robot.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select supermarketbot
source install/setup.bash
```

### Launch

```bash
# Step 1: SLAM — Build the supermarket map
ros2 launch supermarketbot slam.launch.py

# Step 2: Teleop — Drive through all aisles
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

# Step 3: Save map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/supermarketbot/maps/world_map

# Step 4: Nav2 — Autonomous navigation
ros2 launch supermarketbot nav2.launch.py
```

---

## 12. Repository Structure

```
supermarketbot/                        # ROS 2 Python package root
│
├── package.xml                        # Package manifest & dependencies
├── setup.py                           # Python package setup & entry points
├── setup.cfg                          # Script install path
├── requirements.txt                   # Python pip dependencies
│
├── supermarketbot/                    # Python source module
│   ├── __init__.py
│   └── vision.py                      # Customer detection node (HOG+SVM)
│
├── launch/
│   ├── slam.launch.py                 # SLAM mode — mapping the supermarket
│   └── nav2.launch.py                 # Nav2 mode — autonomous navigation
│
├── urdf/
│   └── supermarketbot.xacro           # Robot description (URDF via Xacro)
│
├── meshes/                            # 3D mesh files (STL from Fusion 360)
│   ├── chassis_l1.stl                 # Chassis layer 1
│   ├── chassis_l2.stl                 # Chassis layer 2
│   ├── chassis_l3.stl                 # Chassis layer 3
│   ├── wheel.stl                      # Drive wheel (shared L/R)
│   ├── lidar_base_link.stl            # LiDAR housing
│   └── lidar_scanner_link.stl         # LiDAR scanner head
│
├── world/
│   └── world.sdf                      # Supermarket environment (SDF 1.10)
│
├── config/
│   ├── ros_gz_bridge_gazebo.yaml      # ROS ↔ Gazebo topic bridge
│   ├── nav2_params.yaml               # Nav2 stack parameters
│   ├── async_slam_toolbox_cfg.yaml    # SLAM Toolbox configuration
│   ├── sync_slam_toolbox_cfg.yaml     # SLAM Toolbox (sync mode)
│   └── slam_toolbox_cfg.yaml          # SLAM laser parameters
│
├── maps/
│   ├── world_map.pgm                  # Occupancy grid (regenerate after world change)
│   └── world_map.yaml                 # Map metadata
│
├── rviz/
│   └── slam_cfg.rviz                  # RViz2 display configuration
│
└── test/                              # ament lint tests
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## 13. Troubleshooting

### Customer model not loading

The Fuel model (Casual female) downloads automatically on first launch. If offline:

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"
```

### SLAM map regeneration required

The included `world_map.pgm` must be regenerated for the supermarket world:

```bash
# 1. Launch SLAM mode
ros2 launch supermarketbot slam.launch.py

# 2. Drive through all aisles with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

# 3. Save the new map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/supermarketbot/maps/world_map
```

### GZ_SIM_RESOURCE_PATH

The launch files set this automatically. If running Gazebo manually:

```bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix supermarketbot)/share/supermarketbot
```

### Transform timeout errors

Normal during startup — wait 5–10 seconds for all TF frames to become available.

### Robot spawns and falls

The robot spawns at z=0.65 and drops to the ground plane. This is expected — wait for physics to settle before sending commands.

---

<p align="center">
  Built with ROS 2 · Gazebo Sim · Nav2 · SLAM Toolbox · OpenCV
</p>
