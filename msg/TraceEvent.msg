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

# Producer-vs-consumer direction for request/response-style events (ROB-406).
#
# For pub/sub the event_type already encodes direction (PUBLISH_* = producer,
# TAKE_* = consumer). For services and actions, however, the SAME event_type is
# emitted on both ends of an RPC: a client sending a request and a service
# server taking that request both emit EVENT_SERVICE_REQUEST. The
# CorrelationEngine in robot_agent needs to know which side SEEDS the
# correlation window (the producer / client-send) and which side CORRELATES
# against it (the consumer / server-take). This field carries that distinction.
#
# DIRECTION_UNSPECIFIED (0) is the default and preserves the historical meaning
# for pub/sub events, where direction is already implicit in event_type. Old
# producers that never set this field deserialize as UNSPECIFIED.
uint8 direction
uint8 DIRECTION_UNSPECIFIED = 0   # Direction implicit in event_type (pub/sub) or unknown
uint8 DIRECTION_PRODUCER = 1      # This end SEEDS correlation (client send_request / send_response)
uint8 DIRECTION_CONSUMER = 2      # This end CORRELATES (service take_request / client take_response)
