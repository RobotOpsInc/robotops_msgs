# Diagnostics report published by rmw_robotops every 10 seconds
# Topic: /robotops/diagnostics

builtin_interfaces/Time timestamp

# Node identification
string node_name
string node_namespace
string hostname

# Clock synchronization status
uint8 clock_sync_method
uint8 CLOCK_SYNC_NONE = 0
uint8 CLOCK_SYNC_NTP = 1
uint8 CLOCK_SYNC_PTP = 2
uint8 CLOCK_SYNC_GPS = 3
int64 estimated_skew_ns            # Estimated clock skew from reference
uint64 skew_uncertainty_ns         # Uncertainty in skew estimate
bool clock_sync_acceptable         # True if within acceptable threshold

# Tracing health metrics
uint64 traces_emitted              # Total trace events emitted
uint64 traces_dropped              # Events dropped (queue full)
uint64 correlation_successes       # Successful pub->sub correlations
uint64 correlation_failures        # Failed correlations (no match)
uint64 correlation_ambiguous       # Ambiguous correlations (multiple matches)
float32 queue_utilization          # Current queue fill level (0.0-1.0)
bool tracing_enabled               # Current tracing state
bool auto_disabled                 # True if tracing was auto-disabled due to failures

# DDS capabilities
string rmw_implementation          # e.g., "rmw_fastrtps_cpp"
bool related_sample_identity_supported  # True if deterministic correlation available
string correlation_strategy        # Active strategy name

# Resource usage
uint64 memory_used_bytes           # Approximate memory usage
float32 cpu_percent                # CPU usage of tracing subsystem
