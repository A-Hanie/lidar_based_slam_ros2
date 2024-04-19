cmake_minimum_required(VERSION 3.5)
project(ndt_slam)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp 
  rclcpp_components 
  tf2_ros 
  tf2_geometry_msgs 
  tf2_sensor_msgs 
  tf2_eigen 
  pcl_conversions
  geometry_msgs 
  sensor_msgs
  nav_msgs
  slam_msgs
  ndt_omp_ros2
  std_srvs
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ndt_slam SHARED
src/ndt_slam.cpp
)

ament_target_dependencies(ndt_slam ${dependencies})

add_executable(ndt_slam_node
src/ndt_slam_node.cpp
)

target_link_libraries(ndt_slam_node
ndt_slam ${PCL_LIBRARIES})

ament_target_dependencies(ndt_slam_node
  rclcpp)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})



ament_export_libraries(ndt_slam)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ndt_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ndt_slam_node
  DESTINATION lib/${PROJECT_NAME}
)




ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
