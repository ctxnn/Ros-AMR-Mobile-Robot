# Running `supermarketbot` in Docker

This repo has no `Dockerfile` yet. This guide gives you one plus the run
commands, tailored to what this package actually needs (ROS 2 **Humble**,
`ament_python` build, Gazebo Sim GUI, RViz2, Nav2, SLAM Toolbox).

Because Gazebo Sim and RViz2 are GUI apps, the container needs access to your
host's X server. That's the main wrinkle below.

---

## 1. Dockerfile

Create this at the repo root (`Ros-AMR-Mobile-Robot/Dockerfile`):

```dockerfile
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# System deps this package's README calls for
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir numpy==1.26.4 opencv-python

# Bring in the package sources
WORKDIR /ros2_ws/src
COPY supermarketbot ./supermarketbot

WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --packages-select supermarketbot

# Source both underlay and workspace on every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
```

Notes:
- Only the `supermarketbot/` subfolder is a real ROS package (the `cad/` and
  `demo_videos/` folders aren't needed at runtime, so they're left out of the
  image to keep it small).
- `osrf/ros:humble-desktop` already ships Gazebo-compatible bits, RViz, etc.,
  but the explicit installs above match this repo's documented dependency
  list exactly.

---

## 2. Build the image

From the repo root:

```bash
docker build -t supermarketbot:humble .
```

---

## 3. Enable GUI passthrough (Linux host)

Gazebo Sim and RViz2 need an X server. On a Linux host with X11:

```bash
xhost +local:docker
```

(Run `xhost -local:docker` afterward to revoke access again.)

---

## 4. Run the container

```bash
docker run -it --rm \
  --name supermarketbot \
  --network host \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  supermarketbot:humble
```

- `--network host` keeps ROS 2 DDS discovery simple (no need to fiddle with
  `ROS_DOMAIN_ID`/multicast across container network namespaces).
- `--device /dev/dri` gives the container your GPU for Gazebo's OGRE2
  renderer. If you're on an Nvidia GPU with `nvidia-docker`/NVIDIA Container
  Toolkit installed, add `--gpus all` instead (and drop `/dev/dri` if it
  doesn't apply).
- If the map/config files should persist changes back to your host, add a
  bind mount, e.g. `-v $(pwd)/supermarketbot/maps:/ros2_ws/src/supermarketbot/maps`.

### macOS / Windows hosts

Native GUI passthrough doesn't work the same way:
- **macOS**: install [XQuartz](https://www.xquartz.org/), enable
  "Allow connections from network clients" in its preferences, then run
  `xhost + 127.0.0.1` and set `DISPLAY=host.docker.internal:0` in the
  `docker run` command instead of `$DISPLAY`.
- **Windows (WSL2)**: use WSLg (built into recent Windows 11), which already
  provides `DISPLAY` and `/tmp/.X11-unix` inside WSL — the Linux command above
  works unchanged from a WSL2 shell.

---

## 5. Launch the stack (inside the container)

Each of these opens Gazebo/RViz GUIs, so open a new terminal into the running
container for each step (`docker exec -it supermarketbot bash`).

```bash
# Terminal 1 — SLAM: build the supermarket map
ros2 launch supermarketbot slam.launch.py

# Terminal 2 — drive the robot through all aisles
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/supermarketbot/cmd_vel

# Terminal 3 — save the map once mapping looks complete
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/supermarketbot/maps/world_map

# Terminal 1 (after stopping SLAM) — Nav2 autonomous navigation
ros2 launch supermarketbot nav2.launch.py
```

In RViz: use **2D Pose Estimate** to set the robot's start pose, then
**Nav2 Goal** to send it to a target location.

---

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo/RViz window doesn't appear | Re-check `xhost +local:docker` was run on the host and `DISPLAY` matches inside the container (`echo $DISPLAY`). |
| `Could not open display` | X11 socket not mounted — verify `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` and that `DISPLAY` was passed through. |
| Software (llvmpipe) rendering, very slow | GPU device not passed in — add `--device /dev/dri` (Mesa/Intel/AMD) or `--gpus all` (Nvidia). |
| TF / transform timeout errors at startup | Normal — wait 5–10s for all frames to become available. |
| Robot spawns then falls | Expected; it spawns at `z=0.65` and settles onto the ground plane. |
| Nodes on host can't see container's ROS graph | Make sure `--network host` was used, and that both sides share the same `ROS_DOMAIN_ID` (default 0 is fine if unset on both). |
