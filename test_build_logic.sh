#!/bin/bash
# Test script to validate build.sh logic without requiring full ROS 2 environment
# This helps identify issues before running in devcontainer

echo "=========================================="
echo "Build Script Logic Validator"
echo "=========================================="
echo ""

# Test 1: Check required files
echo "Test 1: Checking required files..."
if [ -f "build.sh" ]; then
    echo "  ✓ build.sh exists"
else
    echo "  ✗ build.sh not found"
    exit 1
fi

if [ -f "deps.repos" ]; then
    echo "  ✓ deps.repos exists"
    echo "    Size: $(wc -c < deps.repos) bytes"
    echo "    Repositories: $(grep -c "type: git" deps.repos)"
else
    echo "  ⚠ deps.repos not found (will be auto-downloaded)"
fi

if [ -f "CMakeLists.txt" ]; then
    echo "  ✓ CMakeLists.txt exists"
else
    echo "  ✗ CMakeLists.txt not found"
fi

echo ""

# Test 2: Check script syntax
echo "Test 2: Validating bash syntax..."
if bash -n build.sh 2>/dev/null; then
    echo "  ✓ build.sh syntax is valid"
else
    echo "  ✗ build.sh has syntax errors"
    bash -n build.sh
    exit 1
fi

echo ""

# Test 3: Check for required commands in build.sh
echo "Test 3: Checking build.sh structure..."
commands=("vcs import" "rosdep install" "colcon build" "source" "apt --fix-broken")
for cmd in "${commands[@]}"; do
    if grep -q "$cmd" build.sh; then
        echo "  ✓ Found: $cmd"
    else
        echo "  ⚠ Not found: $cmd"
    fi
done

echo ""

# Test 4: Validate deps.repos YAML
echo "Test 4: Validating deps.repos format..."
if [ -f "deps.repos" ]; then
    if command -v python3 &> /dev/null; then
        if python3 -c "import yaml; yaml.safe_load(open('deps.repos'))" 2>/dev/null; then
            echo "  ✓ deps.repos is valid YAML"
        else
            echo "  ✗ deps.repos has YAML syntax errors"
            python3 -c "import yaml; yaml.safe_load(open('deps.repos'))"
            exit 1
        fi
    else
        echo "  ⚠ Python3 not available, skipping YAML validation"
    fi
else
    echo "  ⚠ deps.repos not found"
fi

echo ""

# Test 5: Check environment detection logic
echo "Test 5: Testing environment detection..."
if [ -f "/.dockerenv" ]; then
    echo "  ✓ Running in Docker container"
elif [ -n "$REMOTE_CONTAINERS" ]; then
    echo "  ✓ Running in VS Code devcontainer"
else
    echo "  ℹ Not running in devcontainer (expected for this test)"
fi

echo ""

# Test 6: Check tools availability
echo "Test 6: Checking available tools..."
tools=("git" "curl" "bash" "python3")
for tool in "${tools[@]}"; do
    if command -v "$tool" &> /dev/null; then
        echo "  ✓ $tool is available"
    else
        echo "  ✗ $tool not found"
    fi
done

# Optional ROS 2 tools
ros_tools=("vcs" "rosdep" "colcon")
echo ""
echo "  ROS 2 tools (optional for this test):"
for tool in "${ros_tools[@]}"; do
    if command -v "$tool" &> /dev/null; then
        echo "    ✓ $tool is available"
    else
        echo "    ✗ $tool not found (required in devcontainer)"
    fi
done

echo ""

# Test 7: Simulate deps.repos download
echo "Test 7: Testing deps.repos download URL..."
if curl -fsSL --head https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos 2>&1 | grep -q "200 OK"; then
    echo "  ✓ Official ROS 2 repos file is accessible"
else
    echo "  ✗ Cannot access official ROS 2 repos file"
fi

echo ""
echo "=========================================="
echo "Validation Summary"
echo "=========================================="
echo ""
echo "✓ Basic validation passed"
echo "ℹ To fully test the build, run ./build.sh in the devcontainer"
echo ""
echo "If running in devcontainer:"
echo "  1. Ensure ROS 2 is sourced: source /opt/ros/humble/setup.bash"
echo "  2. Run: ./build.sh"
echo "  3. For clean rebuild: ./build.sh clean"
echo ""
