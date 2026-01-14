Changelog
=========

0.4.2 (2026-01-14)
-------------------

* Fixed Cloudsmith api auth in release workflows

0.4.1 (2026-01-14)
-------------------

* Fixed Cargo registry authentication in release workflows
* Fixed missing version in builtin_interfaces dependency during publish

0.4.0 (2026-01-14)
-------------------

**Added**

* **Rust SDK**: Auto-generated Rust bindings for all message types using ros2_rust
* ``robotops-msgs`` crate with TraceEvent, TraceContextChange, DiagnosticsReport, StartupDiagnostics
* ``builtin-interfaces`` bundled dependency for ROS2 Time types
* SDK published to Cloudsmith Cargo registry on release
* ``just generate`` command to regenerate Rust SDK from .msg files

**Changed**

* Dockerfile now includes ros2_rust built from source for Rust code generation
* CI validates generated Rust SDK (cargo check, clippy, test)

0.3.2 (2026-01-12)
-------------------

* Release workflow improvements

0.3.1 (2026-01-12)
------------------

* Added versioning and release workflow infrastructure
* Added justfile with bump-version command
* Added CHANGELOG.rst validation to version-check workflow
* Replaced automatic CD with manual release workflows

0.3.0 (2026-01-10)
------------------

* **Packaging**: Switched from binary to source distribution
* Package is now ``Architecture: all`` (architecture-independent)
* .deb installs message sources to ``/opt/ros/jazzy/share/robotops_msgs/``
* Users compile messages in their workspace with ``colcon build``
* Simplifies CI (single build instead of per-architecture builds)

0.2.0 (2026-01-06)
------------------

**Added**

* **DiagnosticsReport.msg**: New message for health metrics published every 10 seconds on ``/robotops/diagnostics``
* **StartupDiagnostics.msg**: New message for one-time capability report on ``/robotops/startup_diagnostics``
* **TraceEvent.msg**: New fields for enhanced correlation and span reconstruction
  (event_type, source_timestamp_ns, content_hash, msg_ptr, dds_domain_id, correlation_method)

**Changed**

* **TraceEvent.msg**: Replaced ``operation`` field with ``event_type``

**Removed**

* **TraceEvent.msg**: Removed ``operation`` field and all ``OP_*`` constants

0.1.10 (2025-12-01)
-------------------

* Previous stable release

0.1.0 (2025-01-01)
------------------

* Initial release with TraceEvent.msg and TraceContextChange.msg
* Basic distributed tracing support for ROS2 Jazzy
