FROM ros:humble-ros-base

# Define a build-time variable for the package name
ARG PACKAGE_NAME

# Set the same variable as an environment variable if you need it at runtime
ENV PACKAGE_NAME=${PACKAGE_NAME}

# Update and install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create a workspace directory
WORKDIR /ros2_ws


# Create the C++ package
RUN mkdir src && \
    . /opt/ros/humble/setup.sh && \
    cd src && \
    ros2 pkg create --build-type ament_cmake ${PACKAGE_NAME}