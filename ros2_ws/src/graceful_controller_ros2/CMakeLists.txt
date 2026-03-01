cmake_minimum_required(VERSION 3.16)
project(graceful_controller_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CONTROLLER_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

set(CONTROLLER_SOURCES
  ${CONTROLLER_ROOT}/graceful_controller_standalone.cpp
)

add_library(graceful_controller_core STATIC ${CONTROLLER_SOURCES})
target_include_directories(graceful_controller_core PUBLIC
  ${CONTROLLER_ROOT}
)

add_executable(graceful_controller_ros2_node
  src/graceful_controller_ros2.cpp
)
target_include_directories(graceful_controller_ros2_node PUBLIC
  ${CONTROLLER_ROOT}
)
ament_target_dependencies(graceful_controller_ros2_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(graceful_controller_ros2_node graceful_controller_core)

install(TARGETS graceful_controller_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 
  ${CONTROLLER_ROOT}/config/controller_params.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/publish_offline_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
