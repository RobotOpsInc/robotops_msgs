# robotops_msgs development and build image
# Supports: ROS2 message builds, Debian packaging, and Rust SDK generation
#
# Usage:
#   docker build -t robotops_msgs .
#   docker run --rm robotops_msgs bash -c "just generate"

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

# Install build, packaging, and Rust SDK generation dependencies
RUN apt-get update && apt-get install -y \
    # ROS2 build tools
    python3-colcon-common-extensions \
    python3-bloom \
    # Debian packaging
    fakeroot \
    dpkg-dev \
    debhelper \
    # Rust SDK generation (ros2_rust from source)
    git \
    libclang-dev \
    python3-pip \
    python3-vcstool \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Rust with clippy
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --component clippy
ENV PATH="/root/.cargo/bin:${PATH}"

# Install colcon cargo plugins for ros2_rust
RUN pip3 install --break-system-packages \
    git+https://github.com/colcon/colcon-cargo.git \
    git+https://github.com/colcon/colcon-ros-cargo.git

# Set up workspace
WORKDIR /ws

# Clone ros2_rust and import dependencies for Rust message generation
ARG ROS_DISTRO=jazzy
RUN mkdir -p src \
    && git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust \
    && vcs import src < src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos

# Build ros2_rust (message generator + runtime) and builtin_interfaces with Rust bindings
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-up-to rosidl_generator_rs builtin_interfaces

# Copy package files
WORKDIR /ws/src/robotops_msgs
COPY . .

# Build robotops_msgs (ROS2 + Rust bindings)
WORKDIR /ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && . /ws/install/setup.sh \
    && colcon build --packages-select robotops_msgs

# Source workspace on container start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /ws/install/setup.bash" >> ~/.bashrc

WORKDIR /ws/src/robotops_msgs
