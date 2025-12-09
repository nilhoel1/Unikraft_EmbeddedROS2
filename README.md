# ROS 2 on Unikraft with app-elfloader

This repository demonstrates running ROS 2 Humble on Unikraft using the app-elfloader. The application features a LifecycleNode with MultiThreadedExecutor, built as a static PIE binary for seamless integration with Unikraft's lightweight unikernel architecture.

## Features

- **ROS 2 Humble**: Latest LTS release of ROS 2
- **Lifecycle Node**: Implements ROS 2 lifecycle management
- **MultiThreadedExecutor**: Enables concurrent callback execution
- **Static PIE Binary**: Built with `-static-pie` for Unikraft compatibility
- **Unikraft app-elfloader**: Loads and executes the ROS 2 binary
- **lwip Networking**: Lightweight IP stack for network communication
- **POSIX Scheduler**: Enables multi-threaded execution in Unikraft

## Architecture

```
┌─────────────────────────────────────────┐
│     ROS 2 Application (main.cpp)        │
│  - LifecycleNode                        │
│  - MultiThreadedExecutor (4 threads)    │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│  Static PIE Binary (unikraft_ros2_node) │
│  - Statically linked RMW & libraries    │
│  - No dynamic loading (no dlopen)       │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│      Unikraft app-elfloader             │
│  - Loads PIE binary from initrd         │
│  - lwip (networking)                    │
│  - posix-scheduler (threading)          │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│        QEMU / Hardware                   │
└─────────────────────────────────────────┘
```

## Prerequisites

- Docker (for devcontainer)
- VS Code with Remote-Containers extension (optional)

## Quick Start

### Option 1: Using DevContainer (Recommended)

1. Open this repository in VS Code
2. When prompted, click "Reopen in Container"
3. Wait for the container to build (includes ROS 2, KraftKit, QEMU)
4. Inside the container, run:
   ```bash
   ./build.sh
   ```

### Option 2: Manual Setup

If you prefer not to use devcontainers:

1. Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation.html
2. Install KraftKit: https://unikraft.org/docs/cli/
3. Install QEMU: `sudo apt-get install qemu-system-x86`
4. Run the build script:
   ```bash
   ./build.sh
   ```

## Build Process

The `build.sh` script performs the following steps:

1. **Check environment and tools**:
   - Detect if running in devcontainer
   - Verify vcs and rosdep tools are available
   - Source ROS 2 environment (`/opt/ros/humble/setup.bash`)
2. **Set up local ROS 2 workspace** (first run only):
   - Create `deps/` workspace directory
   - Use `vcs` to import ROS 2 source packages (rclcpp, rcl, rmw, etc.) from `deps.repos`
   - Run `rosdep install` to ensure all dependencies are met
3. **Build dependencies statically** (first run only):
   - Build the `deps/` workspace with `-DBUILD_SHARED_LIBS=OFF` for static linking
   - Source the built deps workspace
4. **Build the main package**:
   - Compile the ROS 2 node as a static PIE binary
   - Link all RMW libraries statically to avoid dlopen
5. **Prepare initrd**:
   - Copy the binary to `.unikraft/initrd/`
6. **Build Unikraft kernel**:
   - Configure app-elfloader with lwip and posix-scheduler
   - Build the unikernel
7. **Run with kraft**:
   - Launch QEMU with the Unikraft kernel
   - Load the ROS 2 binary from initrd

## File Structure

```
.
├── .devcontainer/
│   ├── devcontainer.json    # VS Code devcontainer configuration
│   └── Dockerfile           # Container with ROS 2 + KraftKit + QEMU
├── deps/                    # Local ROS 2 workspace (built statically, auto-generated)
│   ├── src/                 # ROS 2 source packages (from deps.repos)
│   ├── build/               # Build artifacts for dependencies
│   └── install/             # Statically built ROS 2 libraries
├── src/
│   └── main.cpp            # ROS 2 LifecycleNode implementation
├── CMakeLists.txt          # Build configuration with -static-pie
├── package.xml             # ROS 2 package metadata
├── deps.repos              # vcstool configuration for ROS 2 dependencies
├── kraft.yaml              # Unikraft configuration (app-elfloader, lwip, posix)
├── build.sh                # Build and run script
└── README.md               # This file
```

## Configuration Details

### CMakeLists.txt

Critical configuration for static PIE build:

```cmake
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-pie")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")
set(BUILD_SHARED_LIBS OFF)
```

This ensures:
- All libraries are linked statically
- Binary is position-independent (required for app-elfloader)
- No dynamic loading (avoids dlopen issues in Unikraft)

### kraft.yaml

Key Unikraft configurations:

- **app-elfloader**: Loads the static PIE binary
- **lwip**: Provides networking stack
- **posix-scheduler**: Enables multi-threading
- **VFS & ramfs**: Filesystem support for initrd

## Lifecycle States

The ROS 2 node supports the standard lifecycle transitions:

1. **Unconfigured** → `configure()` → **Inactive**
2. **Inactive** → `activate()` → **Active**
3. **Active** → `deactivate()` → **Inactive**
4. **Inactive** → `cleanup()` → **Unconfigured**
5. Any state → `shutdown()` → **Finalized**

The node automatically transitions through configure and activate on startup.

## Customization

### Modify the Node

Edit `src/main.cpp` to customize:
- Node name and behavior
- Publishers/Subscribers
- Timer callbacks
- Lifecycle transitions

### Adjust Thread Count

In `src/main.cpp`, modify the MultiThreadedExecutor:

```cpp
auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
  rclcpp::ExecutorOptions(),
  4  // Change this number
);
```

### Network Configuration

Modify `kraft.yaml` to adjust network settings:
- DHCP vs static IP
- Bridge vs NAT mode
- Additional network interfaces

### Memory Allocation

Adjust memory in `build.sh`:

```bash
kraft run --initrd .unikraft/initrd --memory 1G  # Increase to 1GB
```

## Troubleshooting

### Build Fails with "Shared library not found" Errors

The script now builds ROS 2 libraries from source in the `deps/` workspace to support static linking. If you encounter issues:

1. Clean and rebuild:
   ```bash
   ./build.sh clean
   ./build.sh
   ```

2. Ensure vcstool and rosdep are installed:
   ```bash
   sudo apt-get install python3-vcstool python3-rosdep
   ```

### Build Fails with Linking Errors

Ensure all ROS 2 dependencies are installed:
```bash
rosdep install --from-paths deps/src src --ignore-src -r -y
```

### Binary Not Loaded in Unikraft

Verify the binary is static PIE:
```bash
file install/unikraft_ros2/lib/unikraft_ros2/unikraft_ros2_node
# Should show: "ELF 64-bit LSB pie executable"
```

### Network Not Working

Check QEMU network configuration:
```bash
# Verify bridge exists
ip link show virbr0

# Or use different network mode
kraft run --network nat
```

### Insufficient Memory

Increase memory allocation:
```bash
kraft run --memory 1G --initrd .unikraft/initrd
```

## Development Workflow

1. Make changes to `src/main.cpp`
2. Rebuild: `./build.sh`
3. Test in Unikraft QEMU environment
4. Iterate

For faster iteration during development:
```bash
# Build only the ROS 2 node (skip Unikraft rebuild)
colcon build --packages-select unikraft_ros2

# Then manually run with kraft
kraft run --initrd .unikraft/initrd
```

## Performance Considerations

- **Memory**: Unikernels use significantly less memory than traditional VMs
- **Boot Time**: Unikraft boots in milliseconds
- **Footprint**: Static linking increases binary size but eliminates runtime dependencies
- **Networking**: lwip provides a lightweight alternative to full Linux networking

## Contributing

Contributions are welcome! Please ensure:
- Code follows ROS 2 style guidelines
- Static PIE compatibility is maintained
- Unikraft configuration remains minimal

## License

Apache-2.0

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Unikraft Documentation](https://unikraft.org/docs/)
- [KraftKit CLI](https://unikraft.org/docs/cli/)
- [app-elfloader](https://github.com/unikraft/app-elfloader)

## Acknowledgments

This project combines:
- **ROS 2 Humble** by Open Robotics
- **Unikraft** unikernel platform
- **lwip** lightweight TCP/IP stack
- **app-elfloader** for binary loading