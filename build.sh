#!/bin/bash
# Build script for ROS 2 on Unikraft using app-elfloader
set -e

echo "=========================================="
echo "Building ROS 2 Node for Unikraft"
echo "=========================================="

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
    rm -rf build install log
    echo "✓ Cleaned build directories"
fi

# Build using colcon with static linking flags
echo ""
echo "Building static PIE binary with colcon..."
colcon build \
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
