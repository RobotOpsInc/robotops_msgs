# robotops_msgs

ROS2 message package for distributed tracing in the RobotOps platform.

## Overview

This package defines custom message types used by `rmw_robotops` to emit trace events that the Robot Agent consumes. Part of the distributed tracing system (ROB-33).

## Messages

### TraceEvent

Primary message for trace context propagation. Published on `/robotops/trace_events`.

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | Event timestamp |
| `trace_id` | `string` | UUID shared across related spans |
| `span_id` | `string` | UUID unique to this span |
| `parent_span_id` | `string` | Parent span UUID (empty if root) |
| `span_links` | `string[]` | Fan-in links, format: `"trace_id:span_id"` |
| `operation` | `uint8` | Operation type (see constants) |
| `topic_or_service` | `string` | Topic or service name |
| `node_name` | `string` | Fully qualified node name |
| `node_namespace` | `string` | Node namespace |
| `publisher_gid` | `string` | DDS publisher GUID (hex) |
| `sequence_number` | `uint64` | Message sequence number |
| `message_type` | `string` | e.g., `sensor_msgs/msg/Image` |
| `message_size_bytes` | `uint32` | Serialized message size |

**Operation Constants:**
- `OP_PUBLISH = 1`
- `OP_SUBSCRIBE = 2`
- `OP_SERVICE_REQUEST = 3`
- `OP_SERVICE_RESPONSE = 4`
- `OP_ACTION_GOAL_SENT = 5`
- `OP_ACTION_GOAL_RECEIVED = 6`
- `OP_ACTION_FEEDBACK = 7`
- `OP_ACTION_RESULT = 8`
- `OP_ACTION_CANCEL = 9`
- `OP_ACTION_GOAL_REJECTED = 10`

### TraceContextChange

Emitted when trace context changes for log correlation. Published on `/robotops/trace_context`.

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | Context change timestamp |
| `node_name` | `string` | Node name |
| `node_namespace` | `string` | Node namespace |
| `thread_id` | `uint64` | Thread ID (for multi-threaded executors) |
| `trace_id` | `string` | Current trace ID (empty = cleared) |
| `span_id` | `string` | Current span ID (empty = cleared) |
| `change_type` | `uint8` | Change type (see constants) |

**Change Type Constants:**
- `CONTEXT_ENTERED = 1` - Callback started with trace context
- `CONTEXT_EXITED = 2` - Callback completed, context cleared

## Topic Configuration

| Topic | Message Type | QoS |
|-------|--------------|-----|
| `/robotops/trace_events` | `TraceEvent` | Reliable, depth=1000 |
| `/robotops/trace_context` | `TraceContextChange` | Reliable, depth=100 |

## Installation

### Production

```bash
# Add RobotOps apt repository
curl -1sLf 'https://dl.cloudsmith.io/public/robotops/robotops/setup.deb.sh' | sudo bash

# Install the package
sudo apt install ros-jazzy-robotops-msgs
```

### Development

```bash
# Add RobotOps development apt repository
curl -1sLf 'https://dl.cloudsmith.io/public/robotops/robotops-development/setup.deb.sh' | sudo bash

# Install the package
sudo apt install ros-jazzy-robotops-msgs
```

## Development (Container-based)

ROS2 Jazzy doesn't have native macOS support. Use Docker for local development:

```bash
# Enter development container
docker compose run --rm dev

# Inside container - build the package
colcon build --packages-select robotops_msgs
source install/setup.bash

# Verify bindings
ros2 interface show robotops_msgs/msg/TraceEvent
ros2 interface show robotops_msgs/msg/TraceContextChange
python3 -c "from robotops_msgs.msg import TraceEvent, TraceContextChange; print('OK')"
```

Build artifacts are cached in Docker volumes for fast rebuilds.

## Building Debian Package

```bash
# Inside container
cd /ws/src/robotops_msgs
dpkg-buildpackage -us -uc -b

# Package will be at /ws/src/ros-jazzy-robotops-msgs_*.deb
```

## Regenerating Debian Packaging

If package.xml changes:

```bash
cd /ws/src/robotops_msgs
bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy
```

## License

Apache-2.0
