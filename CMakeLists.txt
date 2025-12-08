cmake_minimum_required(VERSION 3.8)
project(unikraft_ros2)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# CRITICAL: Set static PIE flags for Unikraft compatibility
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-pie")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")

# Force static linking
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_FIND_LIBRARY_SUFFIXES .a)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(unikraft_ros2_node src/main.cpp)

# Include directories
target_include_directories(unikraft_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set compile features
target_compile_features(unikraft_ros2_node PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(unikraft_ros2_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
)

# CRITICAL: Link statically to avoid dlopen issues in Unikraft
# Link RMW implementation and other libraries statically
target_link_libraries(unikraft_ros2_node
  -static-libgcc
  -static-libstdc++
  -Wl,--whole-archive
  -Wl,--no-whole-archive
  -pthread
  -lrt
  -ldl
)

# Set linker flags to ensure static linking
set_target_properties(unikraft_ros2_node PROPERTIES
  LINK_FLAGS "-static-pie"
  POSITION_INDEPENDENT_CODE ON
)

# Install targets
install(TARGETS unikraft_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
