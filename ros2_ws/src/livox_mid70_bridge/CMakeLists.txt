cmake_minimum_required(VERSION 3.8)
project(livox_mid70_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# First-generation Livox-SDK (Mid-70 speaks the gen-1 protocol; SDK2 does not
# support it). Built locally at ~/lidar_official/Livox-SDK.
set(LIVOX_SDK_ROOT "$ENV{HOME}/lidar_official/Livox-SDK")
set(LIVOX_SDK_INCLUDE "${LIVOX_SDK_ROOT}/sdk_core/include")
set(LIVOX_SDK_STATIC "${LIVOX_SDK_ROOT}/build/sdk_core/liblivox_sdk_static.a")

if(NOT EXISTS "${LIVOX_SDK_STATIC}")
  message(FATAL_ERROR
    "Gen-1 Livox-SDK static lib not found at ${LIVOX_SDK_STATIC}. "
    "Build it first: cd ${LIVOX_SDK_ROOT}/build && cmake .. && make -j4")
endif()

add_executable(mid70_bridge_node src/mid70_bridge_node.cpp)
target_include_directories(mid70_bridge_node PRIVATE ${LIVOX_SDK_INCLUDE})
target_link_libraries(mid70_bridge_node
  ${LIVOX_SDK_STATIC}
  Threads::Threads
)
ament_target_dependencies(mid70_bridge_node
  rclcpp
  sensor_msgs
)

install(TARGETS mid70_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
