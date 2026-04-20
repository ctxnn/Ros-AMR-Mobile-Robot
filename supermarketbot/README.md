# supermarketbot

Autonomous supermarket assistant robot — ROS 2 simulation package.

## Quick Start

```bash
# SLAM mode (build the map)
ros2 launch supermarketbot slam.launch.py

# Teleop (in a second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

# Nav2 mode (autonomous navigation with pre-built map)
ros2 launch supermarketbot nav2.launch.py
```

## Environment Variable

If running Gazebo manually, set the resource path:

```bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix supermarketbot)/share/supermarketbot
```