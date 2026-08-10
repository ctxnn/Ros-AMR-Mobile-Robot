# `supermarketbot` in NVIDIA Isaac Sim

This is a **separate** simulation path from the Gazebo setup in the rest of
this repo — nothing under `supermarketbot/` or `Dockerfile`/`DOCKER.md` is
touched or required by this. The Gazebo world/launch files stay exactly as
they are; you just won't be launching them anymore if you're running this
path instead.

The idea: keep using your existing robot description (`supermarketbot.xacro`)
and ROS 2 stack (SLAM Toolbox, Nav2, `auto_explorer`) unchanged, but swap
Gazebo out for Isaac Sim as the physics/rendering/sensor backend, and swap
your hand-built `world.sdf` out for Isaac Sim's built-in **Warehouse**
environment (much bigger, pre-built shelving/aisles).

I can't run Isaac Sim myself in this environment (no GPU/display here), so
this is a guide for you to follow rather than something I could test
end-to-end — treat the exact menu paths as approximate for whatever Isaac Sim
version you install, and check NVIDIA's docs linked below if something's
moved.

---

## 1. Prerequisites

| Requirement | Notes |
|---|---|
| NVIDIA GPU | RTX-series strongly recommended (RTX renderer + RTX Lidar need it) |
| GPU driver | Recent NVIDIA driver with Vulkan/RTX support |
| OS | Ubuntu 22.04 (matches your existing ROS 2 Humble setup) |
| ROS 2 | Humble — same install you already have for Gazebo |
| Disk space | 20+ GB for Isaac Sim + asset cache |

## 2. Install Isaac Sim

Two supported install paths — pick one:

- **Omniverse Launcher (GUI installer)** — the traditional route, installs
  Isaac Sim as an Omniverse "app". See
  [Isaac Sim Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html).
- **Pip install (`isaacsim` package)** — newer, lighter-weight route that
  installs Isaac Sim into a Python virtual environment, no launcher GUI
  needed. See
  [Isaac Sim Pip Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html).

Either way, follow NVIDIA's current docs for exact version-specific steps —
this changes between releases more often than the rest of this guide.

## 3. Enable the ROS 2 Bridge and URDF Importer extensions

Once Isaac Sim launches:

1. `Window → Extensions`
2. Search for and enable:
   - **ROS 2 Bridge** (`isaacsim.ros2.bridge`)
   - **URDF Importer** (`isaacsim.asset.importer.urdf`)

**Important:** source your ROS 2 Humble environment *before* launching Isaac
Sim (`source /opt/ros/humble/setup.bash`), so the ROS 2 Bridge extension can
find your ROS 2 install and `ROS_DOMAIN_ID` matches the rest of your stack.

Reference: [ROS 2 — Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_landing_page.html)

## 4. Convert the xacro to plain URDF

Isaac Sim's importer wants a `.urdf` file, not `.xacro`. Expand it once,
writing the output into this folder so nothing under `supermarketbot/` is
touched:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run xacro xacro \
  ~/ros2_ws/src/supermarketbot/urdf/supermarketbot.xacro \
  -o ~/ros2_ws/src/isaac_sim/supermarketbot_isaac.urdf
```

The `<gazebo>` blocks in the xacro (DiffDrive plugin, Sensors plugin,
JointStatePublisher plugin, `gz_frame_id` tags) are Gazebo/`gz-sim`-specific
and not part of the URDF spec — Isaac Sim's importer will simply ignore them.
You'll rebuild the equivalent behavior (diff-drive control, LiDAR, joint
state + odom publishing) using Isaac Sim's own ROS 2 Action Graph nodes in
step 6, instead of those plugin blocks.

## 5. Import the robot and load the Warehouse environment

1. `File → Import` → select `supermarketbot_isaac.urdf`.
   - **Uncheck** "Fix Base Link" (the robot is mobile, not a fixed-base arm).
   - Leave "Import Inertia Tensor" checked (your xacro already defines
     per-link inertia).
2. Open the Content Browser (`Window → Browsers → Content`) and load one of
   the built-in warehouse scenes:
   - `Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd` —
     closer to a retail/aisle layout (recommended starting point).
   - `Isaac/Environments/Simple_Warehouse/full_warehouse.usd` — bigger,
     more industrial (forklifts, pallets).
3. Drag the imported robot prim into the scene and reposition it (Move tool)
   into an open aisle, a few cm above the floor so it drops cleanly under
   gravity — same idea as the `z=0.65` spawn height you're using in Gazebo.

Reference: [Environment Assets — Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_environments.html)

## 6. Wire up ROS 2 via Action Graph

`Create → Visual Scripting → Action Graph`, then add nodes to reproduce what
`ros_gz_bridge_gazebo.yaml` did in Gazebo — same topic names, so the rest of
your ROS 2 stack (SLAM Toolbox, Nav2, `auto_explorer`) needs zero changes:

| Purpose | Node(s) | Topic (match existing config) |
|---|---|---|
| Sim clock | `ROS2 Publish Clock` | `/clock` |
| Drive command in | `ROS2 Subscribe Twist` → `Differential Controller` (wheel radius `0.035`, wheel base `0.205`, matching your xacro) → `Articulation Controller` | `/supermarketbot/cmd_vel` |
| Odometry out | `ROS2 Publish Odometry` | `/odom` |
| TF out | `ROS2 Publish Raw Transform Tree` (or the odometry node's built-in TF output) | `/tf` |
| Joint states | `ROS2 Publish Joint State` | `/joint_states` |
| LiDAR | RTX Lidar sensor (`Create → Isaac → Sensors → RTX Lidar`, parented to the `lidar_link` prim) → `ROS2 Publish Laser Scan` | `/scan` |

Trigger all of these off a single `On Playback Tick` node (plus `ROS2
Context` for the domain/QoS settings), same pattern as the differential-drive
tutorial linked below.

Reference: [ROS 2 Reference Architecture](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_reference_architecture.html), [Tutorial: Differential Drive with Action Graph](https://www.youtube.com/watch?v=WqNkmz0BnzQ)

The RTX Lidar's exact FOV/sample-count config lives in a JSON sensor profile
rather than the SDF-style `<lidar>` block you're used to — start from one of
Isaac Sim's example rotary-lidar profiles and adjust range/samples to match
your existing `360 samples, 10 Hz, 0.1–10 m` spec as closely as the profile
schema allows.

## 7. Press Play, then run your existing ROS 2 stack

Once the Action Graph is wired and you hit **Play** in Isaac Sim, it starts
publishing `/clock`, `/odom`, `/tf`, `/joint_states`, `/scan` and subscribing
to `/supermarketbot/cmd_vel` — exactly what Gazebo + `ros_gz_bridge` used to
provide. From here, run the ROS 2 side with the launch file in this folder,
which starts `robot_state_publisher` + SLAM Toolbox + Nav2 **without** any
Gazebo/`gz_sim`/spawn nodes:

```bash
ros2 launch isaac_sim/isaac_nav2_slam.launch.py
```

Then, same as before:

```bash
ros2 run supermarketbot auto_explorer
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/supermarketbot/maps/isaac_warehouse_map
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| ROS 2 Bridge extension can't find ROS 2 / no topics appear | Isaac Sim wasn't launched from a shell with `source /opt/ros/humble/setup.bash` already run |
| Robot falls through the floor on import | "Fix Base Link" was left checked, or collision meshes didn't import — check the Articulation's rigid body/collider settings |
| SLAM/Nav2 nodes never see `/scan` or `/odom` | Action Graph isn't playing (hit **Play**, not just loaded) |
| Topics exist but SLAM Toolbox errors on TF | Check `use_sim_time: true` is set consistently — Isaac Sim's `/clock` must actually be ticking (Play pressed) before ROS 2 nodes come up |
