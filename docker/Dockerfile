# syntax=docker/dockerfile:1.6
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Base tools + ROS2 Python tooling and common packages
# Options are needed to build locally
RUN apt-get -o Acquire::http::Pipeline-Depth="0" \
            -o Acquire::http::No-Cache=true \
            -o Acquire::BrokenProxy=true \
    update && \
    apt-get -o Acquire::http::Pipeline-Depth="0" \
            -o Acquire::http::No-Cache=true \
            -o Acquire::BrokenProxy=true \
    install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
      python3.10-venv \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-ackermann-msgs \
      ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-ros2bag \
      ros-${ROS_DISTRO}-tf2-tools ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-cv-bridge \
      git curl nano && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN pip3 install --no-cache-dir -r requirements.txt

# Workspace path
ENV WS=/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

# Adds python paths
RUN python3 -c "import site, pathlib; p=pathlib.Path(site.getsitepackages()[0])/'ros2.pth'; \
p.write_text('/opt/ros/humble/lib/python3.10/site-packages\n/opt/ros/humble/local/lib/python3.10/dist-packages\n')"

# Entrypoint will source ROS and the workspace, then exec your command
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
