cmake_minimum_required(VERSION 3.8)
project(mapless_nav)

# C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# Optional Nav2 dependencies
find_package(nav2_costmap_2d QUIET)
find_package(nav2_msgs QUIET)
find_package(nav2_util QUIET)

# PCL
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# =============================================================================
# C++ Executables
# =============================================================================

# Point Cloud Fusion Node
add_executable(pointcloud_fusion_node
  src/pointcloud_fusion_node.cpp
)
ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  message_filters
)
target_link_libraries(pointcloud_fusion_node
  ${PCL_LIBRARIES}
)

# Depth to PointCloud Node
add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)
ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)
target_link_libraries(depth_to_pointcloud_node
  ${OpenCV_LIBRARIES}
)

# =============================================================================
# Install
# =============================================================================

# C++ executables
install(TARGETS
  pointcloud_fusion_node
  depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python modules
ament_python_install_package(${PROJECT_NAME})

# Python scripts
install(PROGRAMS
  scripts/target_tracker_node.py
  scripts/following_controller_node.py
  scripts/fake_odom_node.py
  scripts/real_odom_node.py
  scripts/pointcloud_to_scan_node.py
  scripts/return_home_node.py
  scripts/setup_livox_sdk.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# RViz configs
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
