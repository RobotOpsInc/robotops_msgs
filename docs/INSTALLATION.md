# Installation Guide

This guide explains how to install `robotops_msgs` on a ROS2 Jazzy system.

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

### 3. Verify installation

If you have ROS2 Jazzy sourced (e.g., `source /opt/ros/jazzy/setup.bash` in your `~/.bashrc`), the package is immediately available:

```bash
# Verify the messages are available
ros2 interface show robotops_msgs/msg/TraceEvent
ros2 interface show robotops_msgs/msg/TraceContextChange
```

## Do I need to modify my bashrc?

**No additional bashrc changes are needed** if you already have ROS2 Jazzy set up.

The package installs to `/opt/ros/jazzy/`, which is included when you source the ROS2 setup file. If you already have this in your `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
```

Then `robotops_msgs` will be available in any new terminal after installation.

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
