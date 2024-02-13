FROM osrf/ros:humble-desktop

# Update and install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create a workspace directory
WORKDIR /ros2_ws/src
COPY drive_tb3 ./drive_tb3

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
                  apt-get update && \
                  rosdep install --from-paths src --ignore-src -r -y && \
                  colcon build"

RUN . /opt/ros/humble/setup.sh
RUN colcon build