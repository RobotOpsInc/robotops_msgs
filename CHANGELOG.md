# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-01-10

### Changed

- **Packaging**: Switched from binary to source distribution
  - Package is now `Architecture: all` (architecture-independent)
  - .deb installs message sources to `/opt/ros/jazzy/share/robotops_msgs/`
  - Users compile messages in their workspace with `colcon build`
  - Simplifies CI (single build instead of per-architecture builds)

## [0.2.0] - 2026-01-06

### Added

- **DiagnosticsReport.msg**: New message for health metrics published every 10 seconds on `/robotops/diagnostics`
  - Clock synchronization status and skew monitoring
  - Tracing health metrics (emitted, dropped, correlation success/failure)
  - DDS capability reporting
  - Resource usage tracking

- **StartupDiagnostics.msg**: New message for one-time capability report on `/robotops/startup_diagnostics`
  - System information (ROS distro, RMW implementation, DDS version)
  - Capability detection (related_sample_identity, sequence numbers, LTTng, ros2_tracing)
  - Clock verification status
  - Configuration summary and recommendations

- **TraceEvent.msg**: New fields for enhanced correlation and span reconstruction
  - `event_type`: Granular event types for hierarchical span reconstruction
  - `source_timestamp_ns`: Publisher timestamp for fallback correlation
  - `content_hash`: xxHash64 for non-FastDDS correlation
  - `msg_ptr`: Message pointer for ros2_tracing correlation
  - `dds_domain_id`: Multi-domain filtering support
  - `correlation_method`: Indicates which correlation strategy was used

### Changed

- **TraceEvent.msg**: Replaced `operation` field with `event_type`
  - Old: `OP_PUBLISH`, `OP_SUBSCRIBE`, `OP_SERVICE_REQUEST`, etc.
  - New: `EVENT_PUBLISH_RMW_START`, `EVENT_PUBLISH_RMW_END`, `EVENT_TAKE_RMW_START`, `EVENT_TAKE_RMW_END`, etc.

### Removed

- **TraceEvent.msg**: Removed `operation` field and all `OP_*` constants

## Migration Guide (0.1.x to 0.2.0)

This is a **breaking change**. Consumers must update to handle the new message schema.

### TraceEvent.msg Changes

1. **Replace `operation` with `event_type`**:
   ```cpp
   // Before (0.1.x)
   if (msg.operation == robotops_msgs::msg::TraceEvent::OP_PUBLISH) { ... }

   // After (0.2.0)
   if (msg.event_type == robotops_msgs::msg::TraceEvent::EVENT_PUBLISH_RMW_START) { ... }
   ```

2. **Operation mapping**:
   | Old (0.1.x) | New (0.2.0) |
   |-------------|-------------|
   | `OP_PUBLISH` | `EVENT_PUBLISH_RMW_START` / `EVENT_PUBLISH_RMW_END` |
   | `OP_SUBSCRIBE` | `EVENT_TAKE_RMW_START` / `EVENT_TAKE_RMW_END` |
   | `OP_SERVICE_REQUEST` | `EVENT_SERVICE_REQUEST` |
   | `OP_SERVICE_RESPONSE` | `EVENT_SERVICE_RESPONSE` |
   | `OP_ACTION_GOAL_SENT` | `EVENT_ACTION_GOAL` |
   | `OP_ACTION_GOAL_RECEIVED` | `EVENT_ACTION_GOAL` |
   | `OP_ACTION_FEEDBACK` | `EVENT_ACTION_FEEDBACK` |
   | `OP_ACTION_RESULT` | `EVENT_ACTION_RESULT` |
   | `OP_ACTION_CANCEL` | `EVENT_ACTION_CANCEL` |
   | `OP_ACTION_GOAL_REJECTED` | *(handle via result status)* |

3. **New required fields** - These fields are always present:
   - `event_type` (uint8)
   - `msg_ptr` (uint64) - may be 0 if not available
   - `dds_domain_id` (uint32)

4. **New optional fields** - May be 0/empty if not computed:
   - `content_hash` (uint64)
   - `source_timestamp_ns` (int64)
   - `correlation_method` (uint8)

### New Message Types

Subscribe to new diagnostic topics for observability:

```cpp
// Health metrics (every 10s)
auto diag_sub = node->create_subscription<robotops_msgs::msg::DiagnosticsReport>(
    "/robotops/diagnostics", rclcpp::QoS(10).reliable(), callback);

// One-time startup report (transient local)
auto startup_sub = node->create_subscription<robotops_msgs::msg::StartupDiagnostics>(
    "/robotops/startup_diagnostics",
    rclcpp::QoS(1).reliable().transient_local(), callback);
```

## [0.1.10] - 2025-12-XX

- Previous stable release

## [0.1.0] - 2025-XX-XX

### Added

- Initial release with TraceEvent.msg and TraceContextChange.msg
- Basic distributed tracing support for ROS2 Jazzy
