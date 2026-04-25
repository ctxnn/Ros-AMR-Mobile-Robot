# Changes & Improvements (ROS 2 Humble Nav2 Fixes)

This document outlines all the modifications made to the `supermarketbot` package to ensure 100% compatibility with ROS 2 Humble and to fix navigation-halting bugs.

## 1. Nav2 Parameter Overhaul (`nav2_params.yaml`)
- **Foxy-to-Humble Migration**: The entire Foxy-era `nav2_params.yaml` was completely ripped out and replaced with the official ROS 2 Humble `nav2_bringup` defaults. This permanently resolved the notorious Behavior Tree `Any::cast failed because it is empty` crash.
- **AMCL Compatibility**: Fixed a syntax error in the AMCL configuration by updating `robot_model_type` to use the correct Humble namespace syntax (`nav2_amcl::DifferentialMotionModel`).
- **TF Tree Alignment**: Updated the `base_frame_id` in `amcl` and the `robot_base_frame` in both costmaps from the default `base_footprint` to `base_link` to match the custom `supermarketbot.xacro` URDF.

## 2. Gazebo & Costmap Collision Fixes
- **Costmap "Robot Seeing Itself" Fix**: Updated `obstacle_min_range` and `raytrace_min_range` in the local and global costmaps from `0.0` to `0.25`. This prevents the robot's laser scanner from hitting its own physical chassis and marking it as a lethal obstacle (which previously caused the planner to abort immediately with a `Collision Ahead` warning).
- **Gazebo SDF Version**: Downgraded `world.sdf` from `<sdf version='1.10'>` to `<sdf version='1.9'>` for compatibility with Gazebo Fortress.
- **Simulation Startup**: Added the `-r` flag to the `gz sim` launch command in `nav2.launch.py` and `slam.launch.py` so the simulation starts unpaused. This allows Nav2 nodes to immediately synchronize with the `/clock` topic and prevents early TF lookup timeouts.

## 3. Product Locator Implementation
- **New Node (`product_locator.py`)**: Created a command-line script that maps natural language queries (e.g., "snacks", "milk") to safe `(x, y)` coordinate waypoints within the supermarket aisles.
- **Entry Point Registration**: Updated `setup.py` to compile the locator script as an executable (`ros2 run supermarketbot locator`).
- **Documentation**: Added the Product Locator instructions to the main `README.md`.

## 4. Vision Node & Topic Corrections
- **Topic Alignment**: Updated the `image` subscriber inside `vision.py` to explicitly subscribe to `/rgb_camera/image` to properly consume the image feed from the simulated Gazebo camera.
- **Documentation**: Appended instructions on how to run `vision.py` and how to view the annotated bounding box feed using RViz or `rqt_image_view`.
- **Package Installation**: Replaced the `supermarketbot` source directory back into the main git repository.
