FROM ros:jazzy

# Install build and packaging dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-bloom \
    fakeroot \
    dpkg-dev \
    debhelper \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ws/src/robotops_msgs

# Copy package files
COPY . .

# Build
WORKDIR /ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select robotops_msgs

# Source workspace on container start
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc
