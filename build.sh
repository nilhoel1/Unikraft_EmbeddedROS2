#!/bin/bash
# Build script for ROS 2 on Unikraft using app-elfloader
set -e

echo "=========================================="
echo "Building ROS 2 Node for Unikraft"
echo "=========================================="

# Check if running in devcontainer
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IN_DEVCONTAINER=false

if [ -n "$REMOTE_CONTAINERS" ] || [ -n "$CODESPACES" ] || [ "$TERM_PROGRAM" = "vscode" ] || [ -f "/.dockerenv" ]; then
    IN_DEVCONTAINER=true
    echo "✓ Running inside devcontainer"
else
    echo "ℹ Running outside devcontainer"
fi

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS 2 Humble environment"
else
    echo "✗ ROS 2 Humble not found at /opt/ros/humble"
    exit 1
fi

# Clean previous builds (optional)
if [ "$1" = "clean" ]; then
    echo "Cleaning previous builds..."
    rm -rf build install log deps
    echo "✓ Cleaned build directories"
fi

# Check if deps directory exists, if not, set it up
if [ ! -d "deps/src" ]; then
    echo ""
    echo "=========================================="
    echo "Setting up local ROS 2 workspace"
    echo "=========================================="
    
    # Create deps workspace
    mkdir -p deps/src
    echo "✓ Created deps workspace"
    
    # Import ROS 2 source packages using vcs
    echo ""
    echo "Importing ROS 2 source packages..."
    if [ ! -f "deps.repos" ]; then
        echo "✗ deps.repos file not found"
        exit 1
    fi
    
    cd deps
    vcs import src < ../deps.repos
    if [ $? -eq 0 ]; then
        echo "✓ Successfully imported ROS 2 source packages"
    else
        echo "✗ Failed to import source packages"
        exit 1
    fi
    cd ..
    
    # Run rosdep install to ensure all dependencies are met
    echo ""
    echo "Installing dependencies with rosdep..."
    if [ "$IN_DEVCONTAINER" = true ]; then
        rosdep update || true
    fi
    
    rosdep install --from-paths deps/src --ignore-src -r -y
    if [ $? -eq 0 ]; then
        echo "✓ Dependencies installed successfully"
    else
        echo "⚠ Some dependencies may not have been installed, continuing..."
    fi
    
else
    echo "✓ deps workspace already exists"
fi

# Build the deps workspace with static linking
if [ ! -f "deps/install/setup.bash" ]; then
    echo ""
    echo "=========================================="
    echo "Building deps workspace statically"
    echo "=========================================="
    
    cd deps
    colcon build \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF \
        --event-handlers console_direct+
    
    if [ $? -eq 0 ]; then
        echo "✓ deps workspace built successfully"
    else
        echo "✗ deps workspace build failed"
        exit 1
    fi
    cd ..
else
    echo "✓ deps workspace already built"
fi

# Source the deps workspace
echo ""
echo "Sourcing deps workspace..."
source deps/install/setup.bash
echo "✓ Sourced deps workspace"

# Build the main package with static linking
echo ""
echo "=========================================="
echo "Building unikraft_ros2 package"
echo "=========================================="
echo "Building static PIE binary with colcon..."
colcon build \
    --packages-select unikraft_ros2 \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_EXE_LINKER_FLAGS="-static-pie" \
    -DCMAKE_CXX_FLAGS="-fPIE" \
    -DBUILD_SHARED_LIBS=OFF

if [ $? -eq 0 ]; then
    echo "✓ Build successful"
else
    echo "✗ Build failed"
    exit 1
fi

# Check if the binary was created
BINARY_PATH="install/unikraft_ros2/lib/unikraft_ros2/unikraft_ros2_node"
if [ ! -f "$BINARY_PATH" ]; then
    echo "✗ Binary not found at $BINARY_PATH"
    exit 1
fi

echo "✓ Binary created at $BINARY_PATH"

# Verify it's a static PIE binary
echo ""
echo "Verifying binary type..."
file "$BINARY_PATH"
ldd "$BINARY_PATH" 2>&1 | head -5 || echo "Static binary (ldd fails as expected)"

# Initialize Unikraft application (if not already done)
echo ""
echo "=========================================="
echo "Preparing Unikraft Environment"
echo "=========================================="

# Check if kraft is available
if ! command -v kraft &> /dev/null; then
    echo "✗ kraft command not found. Please install KraftKit."
    exit 1
fi

echo "✓ kraft is available"

# Create initrd with the binary
echo ""
echo "Creating initrd with ROS 2 binary..."
mkdir -p .unikraft/initrd
cp "$BINARY_PATH" .unikraft/initrd/unikraft_ros2_node
echo "✓ Binary copied to initrd"

# Build Unikraft kernel
echo ""
echo "=========================================="
echo "Building Unikraft Kernel"
echo "=========================================="
kraft build --no-cache --no-update

if [ $? -eq 0 ]; then
    echo "✓ Unikraft kernel built successfully"
else
    echo "✗ Unikraft kernel build failed"
    exit 1
fi

# Run with kraft
echo ""
echo "=========================================="
echo "Running ROS 2 on Unikraft"
echo "=========================================="
echo "Starting Unikraft with app-elfloader..."
echo "Press Ctrl+C to stop"
echo ""

kraft run --initrd .unikraft/initrd --memory 512M --network bridge

echo ""
echo "=========================================="
echo "Execution completed"
echo "=========================================="
