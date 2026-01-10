# Installation Guide

This guide explains how to install `robotops_msgs` on a ROS2 Jazzy system.

## Version Notice

**Current version: 0.3.0** - See [CHANGELOG.md](../CHANGELOG.md) for details.

## Prerequisites

- Ubuntu 24.04 (Noble)
- ROS2 Jazzy installed and sourced

## Installation

### 1. Add the RobotOps apt repository

Run the Cloudsmith setup script to add the repository to your system:

```bash
curl -1sLf 'https://dl.cloudsmith.io/public/robotops/robotops/setup.deb.sh' | sudo bash
```

This adds the RobotOps apt repository and imports the GPG signing key.

### 2. Install the package

```bash
sudo apt update
sudo apt install ros-jazzy-robotops-msgs
```

### 3. Build in your workspace

This package is distributed as source. After installing, build it in your ROS2 workspace:

```bash
cd ~/your_ros2_ws
colcon build --packages-select robotops_msgs
source install/setup.bash
```

### 4. Verify installation

```bash
# Verify the messages are available
ros2 interface show robotops_msgs/msg/TraceEvent
ros2 interface show robotops_msgs/msg/TraceContextChange
ros2 interface show robotops_msgs/msg/DiagnosticsReport
ros2 interface show robotops_msgs/msg/StartupDiagnostics
```

## Workspace setup

The apt package installs message sources to `/opt/ros/jazzy/share/robotops_msgs/`. You must build these in a colcon workspace before use.

If you already have a workspace with `source ~/your_ros2_ws/install/setup.bash` in your `~/.bashrc`, the messages will be available after building.

## Upgrading from 0.1.x

Version 0.2.0 introduces **breaking changes**. Before upgrading:

1. Review the [CHANGELOG.md](../CHANGELOG.md) migration guide
2. Update code that uses `TraceEvent.operation` to use `TraceEvent.event_type`

```bash
sudo apt update
sudo apt install --only-upgrade ros-jazzy-robotops-msgs

# Rebuild in your workspace
cd ~/your_ros2_ws
colcon build --packages-select robotops_msgs
source install/setup.bash
```

## Development Builds

For development/testing builds (not recommended for production):

```bash
# Add the development repository instead
curl -1sLf 'https://dl.cloudsmith.io/public/robotops/robotops-development/setup.deb.sh' | sudo bash

sudo apt update
sudo apt install ros-jazzy-robotops-msgs
```

## Uninstallation

```bash
sudo apt remove ros-jazzy-robotops-msgs
```

To also remove the apt repository:

```bash
sudo rm /etc/apt/sources.list.d/robotops-robotops.list
sudo rm /usr/share/keyrings/robotops-robotops-archive-keyring.gpg
sudo apt update
```
