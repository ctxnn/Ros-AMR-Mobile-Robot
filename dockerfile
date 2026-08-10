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
    ros-${ROS_DISTRO}-topic-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir numpy==1.26.4 opencv-python mujoco

# Bring in the package sources
WORKDIR /ros2_ws/src
COPY supermarketbot ./supermarketbot

WORKDIR /ros2_ws
# rosdep update/install is belt-and-suspenders here — every real dependency is
# already apt-installed explicitly above — so a flaky network hitting GitHub's
# rosdistro raw files shouldn't fail the whole build.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    (rosdep update || true) && \
    (rosdep install --from-paths src --ignore-src -r -y || true) && \
    colcon build --packages-select supermarketbot

# Source both underlay and workspace on every shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]