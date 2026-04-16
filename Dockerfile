# syntax=docker/dockerfile:1.6
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Core build tools + MoveIt2 + Gazebo Harmonic integration + xacro etc.
RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-vcstool \
      ros-dev-tools \
      ros-jazzy-moveit \
      ros-jazzy-moveit-resources \
      ros-jazzy-moveit-servo \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-controller-manager \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Workspace
ENV WS=/ws
WORKDIR ${WS}

# Clone xarm_ros2 (jazzy branch, with submodules)
RUN mkdir -p src && cd src && \
    git clone --recursive -b jazzy \
      https://github.com/xArm-Developer/xarm_ros2.git

# Resolve remaining ROS deps
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
      --rosdistro jazzy

# Build
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Your own package will be bind-mounted at /ws/src/lite6_pick_place via compose.
# After mount, re-run colcon build inside the container once.

# Auto-source on shell entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash" >> /root/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> /root/.bashrc

# WSLg + Gazebo Harmonic rendering fallback — Jazzy's ogre2 hates WSL GPU stacks.
# Uncomment if gz sim segfaults or renders nothing:
# ENV LIBGL_ALWAYS_SOFTWARE=1
# ENV OGRE_RTT_MODE=Copy

CMD ["bash"]