# Timestamp of the event
builtin_interfaces/Time timestamp

# Trace identification
string trace_id                    # UUID, shared across related spans
string span_id                     # UUID, unique to this span
string parent_span_id              # UUID of parent span (empty string if root)

# SpanLinks for fan-in scenarios (multiple inputs → one output)
# Each link is formatted as "trace_id:span_id"
# Example: ["abc123:def456", "ghi789:jkl012"]
string[] span_links

# Event details
uint8 operation                    # See operation constants below
string topic_or_service            # Topic name or service name
string node_name                   # Fully qualified node name (/namespace/node)
string node_namespace              # Node namespace

# For correlation (matching publish to subscribe)
string publisher_gid               # DDS publisher GUID (hex string)
uint64 sequence_number             # Message sequence number within publisher

# Metadata
string message_type                # e.g., "sensor_msgs/msg/Image"
uint32 message_size_bytes          # Serialized message size

# Operation constants
uint8 OP_PUBLISH = 1
uint8 OP_SUBSCRIBE = 2
uint8 OP_SERVICE_REQUEST = 3
uint8 OP_SERVICE_RESPONSE = 4
uint8 OP_ACTION_GOAL_SENT = 5
uint8 OP_ACTION_GOAL_RECEIVED = 6
uint8 OP_ACTION_FEEDBACK = 7
uint8 OP_ACTION_RESULT = 8
uint8 OP_ACTION_CANCEL = 9
uint8 OP_ACTION_GOAL_REJECTED = 10
