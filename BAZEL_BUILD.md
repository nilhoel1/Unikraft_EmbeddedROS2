# Bazel Build Instructions for Static ROS 2 with rmw_zenoh

This directory contains Bazel build files for compiling the ROS 2 node as a static PIE binary with rmw_zenoh middleware.

## Overview

The Bazel build system provides an alternative to the colcon/CMake workflow, with the key advantage of producing a fully static binary where rmw_zenoh is force-registered at link time without requiring `dlopen`.

## Key Components

### 1. BUILD.bazel
Main build file that defines:
- `rmw_zenoh_static_registration`: Library containing the static registration stub
- `unikraft_ros2_node`: Main binary target with static linking

### 2. src/rmw_zenoh_static_stub.cpp
Critical stub code that:
- Sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` environment variable
- Forces strong references to rmw_zenoh symbols
- Prevents linker from removing "unused" code
- Uses `__attribute__((constructor))` to run before main()

### 3. WORKSPACE
Defines external dependencies and paths to ROS 2 libraries in the deps workspace.

### 4. .bazelrc
Configuration file with compiler and linker flags for static PIE builds.

## Prerequisites

1. Build the deps workspace first using colcon:
   ```bash
   ./build.sh  # This builds deps/ with static libraries
   ```

2. Install Bazel:
   ```bash
   # Ubuntu/Debian
   sudo apt install bazel
   
   # Or use Bazelisk (version manager)
   npm install -g @bazel/bazelisk
   ```

## Building

### Standard Build
```bash
bazel build //:unikraft_ros2_node
```

### With Custom Flags
```bash
bazel build //:unikraft_ros2_node \
  --config=release \
  --verbose_failures
```

### Alternative Build (with inline stub)
```bash
bazel build //:unikraft_ros2_node_alt
```

## Running

```bash
# Run directly
bazel run //:unikraft_ros2_node

# Or execute the built binary
./bazel-bin/unikraft_ros2_node
```

## Verifying Static Build

```bash
# Check that it's a static PIE binary
file bazel-bin/unikraft_ros2_node
# Should show: ELF 64-bit LSB pie executable, statically linked

# Verify no dynamic dependencies
ldd bazel-bin/unikraft_ros2_node
# Should show: not a dynamic executable

# Check for rmw_zenoh symbols
nm bazel-bin/unikraft_ros2_node | grep rmw_zenoh
# Should show rmw_zenoh symbols embedded in binary
```

## How Static RMW Registration Works

### Problem
ROS 2 normally uses `dlopen` to dynamically load RMW implementations at runtime. This doesn't work in static binaries or restricted environments like Unikraft.

### Solution
The `rmw_zenoh_static_stub.cpp` file implements several techniques:

1. **Environment Variable Override**:
   ```cpp
   setenv("RMW_IMPLEMENTATION", "rmw_zenoh_cpp", 0);
   ```
   Sets the RMW implementation before rclcpp initialization.

2. **Strong Symbol References**:
   ```cpp
   volatile auto init_ptr = &rmw_init;
   ```
   Forces the linker to include rmw_zenoh functions.

3. **Constructor Attribute**:
   ```cpp
   __attribute__((constructor))
   static void rmw_zenoh_static_init()
   ```
   Runs initialization code before main().

4. **Linker Anchors**:
   ```cpp
   void __rmw_zenoh_static_registration_anchor(void)
   ```
   Provides a symbol that can be referenced by `-Wl,-u` linker flags.

5. **alwayslink = True**:
   Bazel flag that prevents the linker from removing "unused" libraries.

## Troubleshooting

### Issue: "RMW implementation not found"
**Solution**: Ensure the static stub is linked with `alwayslink = True` and linker flags include `--whole-archive`.

### Issue: "Undefined reference to rmw_* symbols"
**Solution**: Check that WORKSPACE points to correct paths in the deps/ directory and that static libraries (.a files) exist.

### Issue: Binary tries to dlopen libraries
**Solution**: Verify that `RMW_IMPLEMENTATION` environment variable is set in the stub's constructor.

### Issue: "Symbol not found" at runtime
**Solution**: Add `-Wl,-u,<symbol_name>` to force-link specific symbols.

## Advanced Configuration

### Custom RMW Implementation Path
Edit WORKSPACE to point to your rmw_zenoh installation:
```python
new_local_repository(
    name = "ros2_rmw_zenoh",
    path = "/custom/path/to/rmw_zenoh",
    ...
)
```

### Additional Linker Flags
Add to .bazelrc:
```
build --linkopt=-Wl,--allow-multiple-definition
build --linkopt=-Wl,--export-dynamic
```

### Cross-Compilation
```bash
bazel build //:unikraft_ros2_node \
  --cpu=x86_64 \
  --crosstool_top=@bazel_tools//tools/cpp:toolchain
```

## Integration with CMake Build

The Bazel build is complementary to the CMake/colcon build:

1. Use `./build.sh` to build the deps workspace with colcon
2. Use Bazel to build the final static binary referencing those deps
3. This gives you the benefits of both build systems

## Notes

- The Bazel build requires the deps workspace to be built first with colcon
- WORKSPACE paths assume the standard layout from `./build.sh`
- The static registration stub ensures rmw_zenoh works without dynamic loading
- All RMW symbols are embedded in the final binary at link time

## Further Reading

- [Bazel C++ Tutorial](https://bazel.build/tutorials/cpp)
- [ROS 2 RMW Documentation](https://docs.ros.org/en/rolling/Concepts/About-ROS-Middleware.html)
- [Zenoh Documentation](https://zenoh.io/docs/)
