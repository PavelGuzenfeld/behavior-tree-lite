cmake_minimum_required(VERSION 3.22)
project(bt_ros2_examples VERSION 0.3.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(behavior_tree_lite REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

add_executable(patrol_robot_node src/patrol_robot_node.cpp)
target_link_libraries(patrol_robot_node behavior_tree_lite::behavior_tree_lite)
ament_target_dependencies(patrol_robot_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
)
install(TARGETS patrol_robot_node RUNTIME DESTINATION lib/${PROJECT_NAME})

find_package(px4_msgs QUIET)
if(px4_msgs_FOUND)
  add_executable(px4_vehicle_node src/px4_vehicle_node.cpp)
  target_link_libraries(px4_vehicle_node behavior_tree_lite::behavior_tree_lite)
  ament_target_dependencies(px4_vehicle_node rclcpp std_msgs px4_msgs)
  install(TARGETS px4_vehicle_node RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "px4_msgs not found, skipping px4_vehicle_node")
endif()

install(DIRECTORY launch rviz urdf DESTINATION share/${PROJECT_NAME})

ament_package()
