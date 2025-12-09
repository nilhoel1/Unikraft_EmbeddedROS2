# Bazel WORKSPACE file for ROS 2 with Zenoh
workspace(name = "unikraft_ros2")

# Load http_archive rule
load("@bazel_tools//tools/build_defs/repo:http_archive.bzl", "http_archive")

# C++ rules
http_archive(
    name = "rules_cc",
    urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.9/rules_cc-0.0.9.tar.gz"],
    sha256 = "2037875b9a4456dce4a79d112a8ae885bbc4aad968e6587dca6e64f3a0900cdf",
    strip_prefix = "rules_cc-0.0.9",
)

# ROS 2 dependencies (these would need to be defined based on your setup)
# In a real scenario, you'd use a tool like bazel-ros2 or define these manually

# Example: Local ROS 2 installation reference
# You would need to adapt these to point to your deps workspace
new_local_repository(
    name = "ros2_rclcpp",
    path = "/workspace/deps/install/rclcpp",
    build_file_content = """
cc_library(
    name = "rclcpp",
    hdrs = glob(["include/**/*.hpp", "include/**/*.h"]),
    includes = ["include"],
    srcs = glob(["lib/librclcpp.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
)

cc_library(
    name = "rclcpp_lifecycle",
    hdrs = glob(["include/**/*.hpp", "include/**/*.h"]),
    includes = ["include"],
    srcs = glob(["lib/librclcpp_lifecycle.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
)
""",
)

new_local_repository(
    name = "ros2_rmw",
    path = "/workspace/deps/install/rmw",
    build_file_content = """
cc_library(
    name = "rmw",
    hdrs = glob(["include/**/*.h", "include/**/*.hpp"]),
    includes = ["include"],
    srcs = glob(["lib/librmw.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
)
""",
)

new_local_repository(
    name = "ros2_rmw_zenoh",
    path = "/workspace/deps/install/rmw_zenoh_cpp",
    build_file_content = """
cc_library(
    name = "rmw_zenoh_cpp",
    hdrs = glob(["include/**/*.h", "include/**/*.hpp"]),
    includes = ["include"],
    srcs = glob(["lib/librmw_zenoh_cpp.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
    alwayslink = True,
)
""",
)

new_local_repository(
    name = "ros2_rcl_interfaces",
    path = "/workspace/deps/install/lifecycle_msgs",
    build_file_content = """
cc_library(
    name = "lifecycle_msgs",
    hdrs = glob(["include/**/*.h", "include/**/*.hpp"]),
    includes = ["include"],
    srcs = glob(["lib/liblifecycle_msgs__rosidl_*.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
)
""",
)

new_local_repository(
    name = "ros2_common_interfaces",
    path = "/workspace/deps/install/std_msgs",
    build_file_content = """
cc_library(
    name = "std_msgs",
    hdrs = glob(["include/**/*.h", "include/**/*.hpp"]),
    includes = ["include"],
    srcs = glob(["lib/libstd_msgs__rosidl_*.a"]),
    visibility = ["//visibility:public"],
    linkstatic = True,
)
""",
)
