#!/bin/bash
# Startup script for running the ROS 2 application with Zenoh middleware
set -e

echo "=========================================="
echo "ROS 2 Application Startup Script"
echo "=========================================="

# Source ROS 2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ Sourced ROS 2 Jazzy environment"
else
    echo "✗ ROS 2 Jazzy not found"
    exit 1
fi

# Source local workspace if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ Sourced local workspace"
elif [ -f "deps/install/setup.bash" ]; then
    source deps/install/setup.bash
    echo "✓ Sourced deps workspace"
fi

# Set RMW implementation to Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
echo "✓ Set RMW_IMPLEMENTATION=rmw_zenoh_cpp"

# Check if binary exists
BINARY_PATH="install/unikraft_ros2/lib/unikraft_ros2/unikraft_ros2_node"
if [ ! -f "$BINARY_PATH" ]; then
    echo "✗ Binary not found at $BINARY_PATH"
    echo "  Please run ./build.sh first to build the application"
    exit 1
fi

echo "✓ Binary found at $BINARY_PATH"

# Run the application
echo ""
echo "=========================================="
echo "Starting ROS 2 Application with Zenoh"
echo "=========================================="
echo ""
echo "Middleware: Zenoh (rmw_zenoh_cpp)"
echo "Node: unikraft_ros2_node (Lifecycle Node)"
echo "Executor: MultiThreadedExecutor (4 threads)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the binary
"$BINARY_PATH"

echo ""
echo "=========================================="
echo "Application stopped"
echo "=========================================="
