# robotops_msgs development commands
# Install just: https://github.com/casey/just

# Default recipe - show available commands
default:
    @just --list

# =============================================================================
# Version Management
# =============================================================================

# Bump version (usage: just bump-version patch|minor|major)
bump-version type:
    #!/usr/bin/env bash
    set -euo pipefail

    if [[ "{{type}}" != "patch" && "{{type}}" != "minor" && "{{type}}" != "major" ]]; then
        echo "Error: type must be 'patch', 'minor', or 'major'"
        exit 1
    fi

    CURRENT=$(grep '<version>' package.xml | sed 's/.*<version>\(.*\)<\/version>.*/\1/')
    IFS='.' read -r major minor patch <<< "$CURRENT"
    DATE=$(date +%Y-%m-%d)

    # Calculate new version
    case "{{type}}" in
        patch)
            NEW_VERSION="$major.$minor.$((patch + 1))"
            ;;
        minor)
            NEW_VERSION="$major.$((minor + 1)).0"
            ;;
        major)
            NEW_VERSION="$((major + 1)).0.0"
            echo "Warning: MAJOR VERSION BUMP: $CURRENT -> $NEW_VERSION"
            echo "Warning: Remember: Major versions must align across all RobotOps components!"
            echo "   - robotops_config"
            echo "   - robotops_msgs"
            echo "   - rmw_robotops"
            echo "   - robot_agent"
            ;;
    esac

    echo "Bumping version: $CURRENT -> $NEW_VERSION"

    # Update package.xml
    sed -i.bak "s|<version>$CURRENT</version>|<version>$NEW_VERSION</version>|" package.xml
    rm package.xml.bak

    # Add changelog entry
    {
        echo "$NEW_VERSION ($DATE)"
        echo "-------------------"
        echo ""
        echo "*"
        echo ""
    } > /tmp/changelog_entry.txt

    # Insert at the top of CHANGELOG.rst (after the header)
    awk '/^[0-9]+\.[0-9]+\.[0-9]+ \(/ { if (!inserted) { system("cat /tmp/changelog_entry.txt"); inserted=1 } } { print }' CHANGELOG.rst > /tmp/CHANGELOG.rst.new
    mv /tmp/CHANGELOG.rst.new CHANGELOG.rst
    rm /tmp/changelog_entry.txt

    echo "Version bumped to $NEW_VERSION"
    echo "Edit CHANGELOG.rst to add your changes"
    if [[ "{{type}}" == "major" ]]; then
        echo "Warning: Coordinate with other RobotOps repos for aligned major version bump!"
    fi

# =============================================================================
# Local Development
# =============================================================================

# Enter development container (ROS2 Jazzy)
dev:
    docker compose run --rm dev

# Build the ROS2 package
build:
    docker compose run --rm dev bash -c "colcon build --packages-select robotops_msgs && source install/setup.bash && ros2 interface show robotops_msgs/msg/TraceEvent"

# Verify Python bindings work
verify:
    docker compose run --rm dev bash -c "colcon build --packages-select robotops_msgs && source install/setup.bash && python3 -c 'from robotops_msgs.msg import TraceEvent, TraceContextChange; print(\"OK\")'"

# Clean build artifacts
clean:
    rm -rf build/ install/ log/
    docker compose down -v

# =============================================================================
# CI Commands (run locally or in GitHub Actions)
# =============================================================================

# Build Docker image for CI
ci-build-image:
    docker build -t robotops_msgs:ci .

# Run full CI suite (build image + tests)
ci: ci-build-image
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running CI suite..."

    echo "Verifying C++ bindings..."
    docker run --rm robotops_msgs:ci bash -c \
        "source /ws/install/setup.bash && ros2 interface show robotops_msgs/msg/TraceEvent"

    echo "Verifying Python bindings..."
    docker run --rm robotops_msgs:ci bash -c \
        "source /ws/install/setup.bash && python3 -c 'from robotops_msgs.msg import TraceEvent, TraceContextChange; print(\"OK\")'"

    echo "Running colcon test..."
    docker run --rm robotops_msgs:ci bash -c \
        "source /opt/ros/jazzy/setup.sh && cd /ws && colcon test --packages-select robotops_msgs && colcon test-result --verbose"

    echo "CI suite passed!"

# Run CI tests only (assumes image already built)
ci-test:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Running CI tests..."

    docker run --rm robotops_msgs:ci bash -c \
        "source /ws/install/setup.bash && ros2 interface show robotops_msgs/msg/TraceEvent"

    docker run --rm robotops_msgs:ci bash -c \
        "source /ws/install/setup.bash && python3 -c 'from robotops_msgs.msg import TraceEvent, TraceContextChange; print(\"OK\")'"

    docker run --rm robotops_msgs:ci bash -c \
        "source /opt/ros/jazzy/setup.sh && cd /ws && colcon test --packages-select robotops_msgs && colcon test-result --verbose"

    echo "CI tests passed!"

# =============================================================================
# Release Commands (build Debian packages)
# =============================================================================

# Build Debian package (outputs to current directory)
build-deb:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Building Debian package..."

    docker run --rm \
        -v "$(pwd)":/output \
        robotops_msgs:ci bash -c "
            cd /ws/src/robotops_msgs && \
            apt-get update && \
            apt-get install -y dpkg-dev fakeroot debhelper && \
            bloom-generate rosdebian --os-name ubuntu --os-version noble --ros-distro jazzy && \
            dpkg-buildpackage -us -uc -b && \
            cp /ws/src/*.deb /output/
        "

    DEB_FILE=$(ls *.deb | head -n1)
    echo "Built package: $DEB_FILE"
