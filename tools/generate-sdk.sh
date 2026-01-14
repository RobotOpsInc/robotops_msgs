#!/bin/bash
# tools/generate-sdk.sh
#
# Generates Rust SDK from .msg files using ros2_rust.
# Extracts generated Rust types from the colcon build output.
# Creates a workspace with robotops-msgs and its builtin_interfaces dependency.
#
# Usage:
#   ./tools/generate-sdk.sh              # Run inside Docker container
#   docker run --rm -v $(pwd)/generated:/output robotops_msgs ./tools/generate-sdk.sh
#
# Output: generated/sdks/rust/

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
OUTPUT_DIR="${PROJECT_DIR}/generated/sdks/rust"

echo "=============================================="
echo "Generating Rust SDK"
echo "=============================================="

# Check if we're in a ROS2 environment
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "Error: ROS2 environment not sourced."
    echo "Run this script inside the Docker container:"
    echo "  docker compose run --rm dev ./tools/generate-sdk.sh"
    exit 1
fi

# Create output directories
mkdir -p "$OUTPUT_DIR/robotops-msgs/src"
mkdir -p "$OUTPUT_DIR/builtin-interfaces/src"

# Get version from package.xml
VERSION=$(grep -oP '(?<=<version>)[^<]+' "$PROJECT_DIR/package.xml")

# ============================================
# Copy robotops_msgs Rust files
# ============================================
ROBOTOPS_RUST_SRC="/ws/install/robotops_msgs/share/robotops_msgs/rust/src"

if [ -d "$ROBOTOPS_RUST_SRC" ]; then
    echo "Copying robotops_msgs Rust files from $ROBOTOPS_RUST_SRC"
    cp -r "$ROBOTOPS_RUST_SRC"/* "$OUTPUT_DIR/robotops-msgs/src/"
else
    echo "Error: Generated Rust files not found at $ROBOTOPS_RUST_SRC"
    echo "Make sure robotops_msgs was built with rosidl_generator_rs available"
    exit 1
fi

# ============================================
# Copy builtin_interfaces Rust files
# ============================================
BUILTIN_RUST_SRC="/ws/install/builtin_interfaces/share/builtin_interfaces/rust/src"

if [ -d "$BUILTIN_RUST_SRC" ]; then
    echo "Copying builtin_interfaces Rust files from $BUILTIN_RUST_SRC"
    cp -r "$BUILTIN_RUST_SRC"/* "$OUTPUT_DIR/builtin-interfaces/src/"
else
    echo "Error: builtin_interfaces Rust files not found at $BUILTIN_RUST_SRC"
    echo "Make sure builtin_interfaces was built with rosidl_generator_rs available"
    exit 1
fi

# ============================================
# Generate workspace Cargo.toml
# ============================================
cat > "$OUTPUT_DIR/Cargo.toml" << 'EOF'
[workspace]
members = ["robotops-msgs", "builtin-interfaces"]
resolver = "2"
EOF

# ============================================
# Generate robotops-msgs Cargo.toml
# ============================================
cat > "$OUTPUT_DIR/robotops-msgs/Cargo.toml" << EOF
[package]
name = "robotops-msgs"
version = "$VERSION"
edition = "2021"
rust-version = "1.75"
license = "Apache-2.0"
repository = "https://github.com/RobotOpsInc/robotops_msgs"
description = "Rust bindings for RobotOps distributed tracing message types"
keywords = ["robotics", "ros2", "tracing", "observability"]
categories = ["robotics", "data-structures"]

[lib]
name = "robotops_msgs"
path = "src/lib.rs"

[dependencies]
rosidl_runtime_rs = "0.4"
builtin_interfaces = { path = "../builtin-interfaces", package = "builtin-interfaces" }
serde = { version = "1.0", features = ["derive"], optional = true }

[features]
default = ["serde"]
serde = ["dep:serde", "rosidl_runtime_rs/serde", "builtin_interfaces/serde"]
EOF

# ============================================
# Generate builtin-interfaces Cargo.toml
# ============================================
cat > "$OUTPUT_DIR/builtin-interfaces/Cargo.toml" << EOF
[package]
name = "builtin-interfaces"
version = "$VERSION"
edition = "2021"
rust-version = "1.75"
license = "Apache-2.0"
repository = "https://github.com/RobotOpsInc/robotops_msgs"
description = "Rust bindings for ROS2 builtin_interfaces types (bundled dependency)"

[lib]
name = "builtin_interfaces"
path = "src/lib.rs"

[dependencies]
rosidl_runtime_rs = "0.4"
serde = { version = "1.0", features = ["derive"], optional = true }

[features]
default = ["serde"]
serde = ["dep:serde", "rosidl_runtime_rs/serde"]
EOF

# ============================================
# Generate README
# ============================================
cat > "$OUTPUT_DIR/README.md" << EOF
# robotops-msgs

Rust bindings for RobotOps distributed tracing message types.

Auto-generated from ROS2 .msg definitions using ros2_rust.

## Installation

1. Configure the Cloudsmith Cargo registry in \`.cargo/config.toml\`:

\`\`\`toml
[registries.robotops]
index = "sparse+https://cargo.cloudsmith.io/robotops/robotops-msgs-rust/"
\`\`\`

2. Add to \`Cargo.toml\`:

\`\`\`toml
[dependencies]
robotops-msgs = { version = "$VERSION", registry = "robotops" }
\`\`\`

## Message Types

| Type | Description | Topic |
|------|-------------|-------|
| \`TraceEvent\` | Primary trace span events | \`/robotops/trace_events\` |
| \`TraceContextChange\` | Log correlation events | \`/robotops/trace_context\` |
| \`DiagnosticsReport\` | Periodic health metrics | \`/robotops/diagnostics\` |
| \`StartupDiagnostics\` | One-time capability report | \`/robotops/startup_diagnostics\` |

## Usage

\`\`\`rust
use robotops_msgs::msg::TraceEvent;

let event = TraceEvent::default();
\`\`\`

## License

Apache-2.0
EOF

echo ""
echo "Rust SDK generated successfully!"
echo "Output: $OUTPUT_DIR"
echo ""
ls -laR "$OUTPUT_DIR"
