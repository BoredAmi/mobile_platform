# jgb_rover simulation: ROS 2 Humble + Gazebo Fortress (Ubuntu 22.04).
# Only the dependencies are installed here; the repository is mounted at /home/dev/mobile and
# built into a named volume (see compose.yaml), so host and container builds never mix.
FROM osrf/ros:humble-desktop

ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz \
      ros-humble-gz-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-robot-localization \
      ros-humble-slam-toolbox \
      ros-humble-nav2-map-server \
      ros-humble-teleop-twist-keyboard \
      ros-humble-xacro \
      ros-humble-joint-state-publisher-gui \
      ros-humble-tf2-tools \
      ros-humble-cv-bridge \
      python3-opencv \
      python3-matplotlib \
      python3-pytest \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# non-root user with the host's UID/GID so files written into the mounted repo stay yours
RUN groupadd --gid ${USER_GID} dev && \
    useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash dev && \
    mkdir -p /home/dev/colcon && chown dev:dev /home/dev/colcon

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/build_ws.sh /usr/local/bin/build_ws
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/build_ws

USER dev
WORKDIR /home/dev/mobile
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
