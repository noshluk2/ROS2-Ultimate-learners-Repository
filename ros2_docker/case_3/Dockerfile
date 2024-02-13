FROM ros:humble-ros-base as stage_1

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



FROM osrf/ros:humble-desktop as stage_2
# Copy the built workspace from the builder stage
COPY --from=stage_1 /ros2_ws/install /ros2_ws/install

# Setup environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo