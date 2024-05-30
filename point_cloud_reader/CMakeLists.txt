cmake_minimum_required(VERSION 3.8)
project(point_cloud_reader)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Needed if you have C++ nodes
find_package(rclpy REQUIRED)   # Needed for Python nodes
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)  # Needed for sensor_msgs dependencies




include_directories(
  include
  ${sensor_msgs_INCLUDE_DIRS}
)

# Python node installation
install(PROGRAMS
  scripts/point_cloud_reader.py
  scripts/pointcloud_reader_service.py
  scripts/pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories such as launch and bags
install(
  DIRECTORY launch bags data
  DESTINATION share/${PROJECT_NAME}
)

# Setup testing configurations
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Include CMake magic to compile ROS nodes
ament_package()
