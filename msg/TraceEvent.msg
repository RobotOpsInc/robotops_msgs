# Timestamp of the event
builtin_interfaces/Time timestamp

# Event type for hierarchical span reconstruction
uint8 event_type
uint8 EVENT_PUBLISH_RMW_START = 1
uint8 EVENT_PUBLISH_RMW_END = 2
uint8 EVENT_TAKE_RMW_START = 3
uint8 EVENT_TAKE_RMW_END = 4
uint8 EVENT_SERVICE_REQUEST = 5
uint8 EVENT_SERVICE_RESPONSE = 6
uint8 EVENT_ACTION_GOAL = 7
uint8 EVENT_ACTION_FEEDBACK = 8
uint8 EVENT_ACTION_RESULT = 9
uint8 EVENT_ACTION_CANCEL = 10

# Trace identification
string trace_id                    # UUID, shared across related spans
string span_id                     # UUID, unique to this span
string parent_span_id              # UUID of parent span (empty string if root)

# SpanLinks for fan-in scenarios (multiple inputs -> one output)
# Each link is formatted as "trace_id:span_id"
# Example: ["abc123:def456", "ghi789:jkl012"]
string[] span_links

# Topic/service identification
string topic_or_service            # Topic name or service name
string node_name                   # Fully qualified node name (/namespace/node)
string node_namespace              # Node namespace

# For correlation (matching publish to subscribe)
string publisher_gid               # DDS publisher GUID (hex string)
uint64 sequence_number             # Message sequence number within publisher
int64 source_timestamp_ns          # Publisher's timestamp for fallback correlation

# Content hash for fallback correlation (non-FastDDS)
uint64 content_hash                # xxHash64 of serialized message (0 if not computed)

# Pointer for span reconstruction (correlate with ros2_tracing)
uint64 msg_ptr                     # Message pointer for event correlation

# Metadata
string message_type                # e.g., "sensor_msgs/msg/Image"
uint32 message_size_bytes          # Serialized message size

# DDS domain for multi-domain filtering
uint32 dds_domain_id

# Correlation metadata
uint8 correlation_method
uint8 CORRELATION_FASTDDS_SEQUENCE = 1    # Deterministic (related_sample_identity)
uint8 CORRELATION_FALLBACK_HASH = 2       # Probabilistic (GID + timestamp + hash)
uint8 CORRELATION_FALLBACK_TIMESTAMP = 3  # Probabilistic (GID + timestamp only)
