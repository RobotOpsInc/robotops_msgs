# Installation Guide

This guide explains how to install `robotops_msgs` on a ROS2 Jazzy system.

## Version Notice

**Current version: 0.5.2** - See [CHANGELOG.md](../CHANGELOG.md) for details.

## Prerequisites

- Ubuntu 24.04 (Noble)
- ROS2 Jazzy installed and sourced

## Installation

### 1. Add the RobotOps apt repository

Add the GPG key and repository to your system:

```bash
# Add the GPG key
curl -fsSL https://apt.robotops.com/robotops-public-key.asc | sudo gpg --dearmor -o /etc/apt/keyrings/robotops.gpg

# Add the repository
echo "deb [signed-by=/etc/apt/keyrings/robotops.gpg] https://apt.robotops.com noble main" | sudo tee /etc/apt/sources.list.d/robotops.list
```

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
# Add the GPG key
curl -fsSL https://apt.development.robotops.com/robotops-public-key.asc | sudo gpg --dearmor -o /etc/apt/keyrings/robotops-dev.gpg

# Add the development repository
echo "deb [signed-by=/etc/apt/keyrings/robotops-dev.gpg] https://apt.development.robotops.com noble main" | sudo tee /etc/apt/sources.list.d/robotops-dev.list

sudo apt update
sudo apt install ros-jazzy-robotops-msgs
```

## Rust SDK (for robot_agent and other Rust consumers)

The `robotops-msgs` Rust crate provides native Rust bindings for all message types.

### 1. Configure AWS CodeArtifact

Authenticate with AWS CodeArtifact:

```bash
# Production
aws codeartifact login --tool cargo \
  --domain robotops \
  --domain-owner 189676910689 \
  --repository robotops-cargo \
  --region us-east-1

# Or for development
aws codeartifact login --tool cargo \
  --domain robotops \
  --domain-owner 717949299175 \
  --repository robotops-cargo \
  --region us-east-1
```

### 2. Add the dependency

Add to your `Cargo.toml`:

```toml
[dependencies]
robotops-msgs = { version = "0.5", registry = "codeartifact" }
```

### 3. Use in your code

```rust
use robotops_msgs::msg::{TraceEvent, TraceContextChange, DiagnosticsReport, StartupDiagnostics};

let event = TraceEvent::default();
```

### Development Builds (Rust)

For development/testing builds, use the development AWS account:

```bash
aws codeartifact login --tool cargo \
  --domain robotops \
  --domain-owner 717949299175 \
  --repository robotops-cargo \
  --region us-east-1
```

```toml
# Cargo.toml
[dependencies]
robotops-msgs = { version = "0.5", registry = "codeartifact" }
```

## Uninstallation

```bash
sudo apt remove ros-jazzy-robotops-msgs
```

To also remove the apt repository:

```bash
sudo rm /etc/apt/sources.list.d/robotops.list
sudo rm /etc/apt/keyrings/robotops.gpg
sudo apt update
```
