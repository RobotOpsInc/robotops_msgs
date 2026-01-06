# One-time diagnostics published at rmw_robotops initialization
# Topic: /robotops/startup_diagnostics

builtin_interfaces/Time timestamp

# System information
string ros_distro                  # e.g., "jazzy"
string rmw_implementation          # e.g., "rmw_fastrtps_cpp"
string dds_version                 # DDS vendor version string
uint32 dds_domain_id

# Capability detection
bool related_sample_identity_supported
bool sequence_number_accessible
bool lttng_available
bool ros2_tracing_available

# Clock verification
uint8 clock_sync_method
int64 initial_clock_skew_ns
bool clock_sync_verified

# Configuration summary
string config_path                 # Path to loaded config file
bool tracing_enabled
float32 default_sample_rate
string[] sampled_topics            # Topics with custom sample rates

# Recommendations (empty if none)
string[] recommendations           # e.g., "Consider FastDDS for optimal correlation"
string[] warnings                  # e.g., "No time synchronization detected"
