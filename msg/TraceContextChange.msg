# Timestamp of context change
builtin_interfaces/Time timestamp

# Node identification
string node_name
string node_namespace

# Thread identification (for multi-threaded executors)
uint64 thread_id

# New trace context (empty strings = context cleared)
string trace_id
string span_id

# Context change type
uint8 change_type
uint8 CONTEXT_ENTERED = 1          # Callback started with trace context
uint8 CONTEXT_EXITED = 2           # Callback completed, context cleared
