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
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ Sourced ROS 2 Jazzy environment"
else
    echo "✗ ROS 2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi

# Check for required tools
if ! command -v vcs &> /dev/null; then
    echo "✗ vcs tool not found. Please install python3-vcstool"
    echo "  You can install it with: sudo apt-get install python3-vcstool"
    exit 1
fi

if ! command -v rosdep &> /dev/null; then
    echo "✗ rosdep not found. Please install python3-rosdep"
    echo "  You can install it with: sudo apt-get install python3-rosdep"
    exit 1
fi

echo "✓ Required tools available (vcs, rosdep)"

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
        echo "⚠ deps.repos file not found, downloading official ROS 2 Jazzy repos file..."
        echo "  Note: This will download many packages and take longer to build."
        echo "  For faster builds, use the curated deps.repos from the repository."
        if curl -fsSL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos -o deps.repos; then
            echo "✓ Downloaded deps.repos from ROS 2 Jazzy repository"
        else
            echo "✗ Failed to download deps.repos"
            exit 1
        fi
    else
        echo "✓ Using existing deps.repos file"
    fi

    cd deps
    echo "Importing repositories into deps/src..."
    if vcs import src < ../deps.repos; then
        echo "✓ Successfully imported ROS 2 source packages"
        echo "  Packages imported:"
        pkg_count=$(ls -1 src/ 2>/dev/null | wc -l)
        ls -1 src/ 2>/dev/null | head -10 | sed 's/^/    - /'
        if [ $pkg_count -gt 10 ]; then
            echo "    ... and $(($pkg_count - 10)) more"
        fi
    else
        echo "✗ Failed to import source packages"
        echo "  Check that vcs is installed and deps.repos is valid"
        exit 1
    fi
    cd ..

    # Run rosdep install to ensure all dependencies are met
    echo ""
    echo "Installing dependencies with rosdep..."

    # Check if rosdep is initialized
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        echo "⚠ rosdep not initialized, initializing..."
        if sudo rosdep init 2>/dev/null; then
            echo "✓ rosdep initialized successfully"
        else
            # Could be already initialized or actual failure - try to continue
            echo "ℹ rosdep init returned non-zero (may already be initialized)"
        fi
    fi

    # Always run rosdep update to ensure sources are current
    echo "Updating rosdep sources..."
    if rosdep update; then
        echo "✓ rosdep updated successfully"
    else
        echo "⚠ rosdep update failed, continuing anyway..."
    fi

    # Fix any broken apt dependencies before rosdep install
    echo ""
    echo "Checking and fixing apt dependencies..."
    if command -v sudo &> /dev/null; then
        if sudo apt --fix-broken install -y; then
            echo "✓ apt dependencies fixed"
        else
            echo "⚠ apt --fix-broken install had issues, continuing..."
        fi

        # Update apt cache to ensure latest package information
        echo "Updating apt package cache..."
        if sudo apt-get update; then
            echo "✓ apt cache updated"
        else
            echo "⚠ apt-get update had issues, continuing..."
        fi
    else
        echo "⚠ sudo not available, skipping apt fixes"
    fi

    if rosdep install --from-paths deps/src --ignore-src -r -y; then
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
    # Allow configuring parallel workers via environment variable
    PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-4}"
    echo "Building deps workspace (this may take 15-30 minutes)..."
    echo "  Using $PARALLEL_WORKERS parallel workers (set COLCON_PARALLEL_WORKERS to override)"
    if colcon build \
        --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DBUILD_SHARED_LIBS=OFF \
        --event-handlers console_direct+ \
        --parallel-workers $PARALLEL_WORKERS; then
        echo "✓ deps workspace built successfully"
    else
        echo "✗ deps workspace build failed"
        echo "  Check the log files in deps/log/ for details"
        exit 1
    fi
    cd ..
else
    echo "✓ deps workspace already built"
fi

# Source the deps workspace
echo ""
echo "Sourcing deps workspace..."
# Unset the system ROS environment to avoid conflicts
if [ -n "$AMENT_PREFIX_PATH" ]; then
    echo "  Clearing system ROS environment variables..."
    unset AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH
    unset COLCON_PREFIX_PATH
    unset LD_LIBRARY_PATH
    unset PATH
    unset PYTHONPATH
    # Restore essential PATH
    export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
fi
source deps/install/setup.bash
echo "✓ Sourced deps workspace (system ROS cleared)"

# Set RMW implementation to Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
echo "✓ Set RMW_IMPLEMENTATION=rmw_zenoh_cpp"

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
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_PREFIX_PATH="$(pwd)/deps/install"

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
