cmake_minimum_required(VERSION 3.22)
project(behavior_tree_lite VERSION 0.0.1 LANGUAGES CXX)

# ==========================================
# OPTIONS
# ==========================================
option(BUILD_TESTING "Build unit tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)

# ==========================================
# C++ STANDARD
# ==========================================
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==========================================
# ROS2 DEPENDENCIES (OPTIONAL)
# ==========================================
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)

set(AMENT_CMAKE_FOUND ${ament_cmake_FOUND})

# ==========================================
# HEADER-ONLY LIBRARY
# ==========================================
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

# ==========================================
# COMPILER WARNINGS
# ==========================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} INTERFACE
    -Wall -Wextra -Wpedantic -Werror=return-type
  )
endif()

# ==========================================
# INSTALL
# ==========================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# ==========================================
# CMAKE PACKAGE CONFIG
# ==========================================
include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# ==========================================
# TESTING
# ==========================================
if(BUILD_TESTING)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest QUIET)
    if(ament_cmake_gtest_FOUND)
      ament_add_gtest(${PROJECT_NAME}_test
        test/test_types.cpp
        test/test_composite.cpp
        test/test_decorator.cpp
        test/test_leaf.cpp
        test/test_integration.cpp
        test/test_dsl.cpp
      )
      target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    endif()
  else()
    # Standalone GTest
    find_package(GTest QUIET)
    if(GTest_FOUND)
      enable_testing()
      add_executable(${PROJECT_NAME}_test
        test/test_types.cpp
        test/test_composite.cpp
        test/test_decorator.cpp
        test/test_leaf.cpp
        test/test_integration.cpp
        test/test_dsl.cpp
      )
      target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}
        GTest::gtest
        GTest::gtest_main
      )
      add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
    endif()
  endif()
endif()

# ==========================================
# EXAMPLES
# ==========================================
if(BUILD_EXAMPLES)
  # Standalone examples (no ROS2 required)
  add_executable(example_robot examples/robot_example.cpp)
  target_link_libraries(example_robot ${PROJECT_NAME})
  
  add_executable(example_dsl examples/minimal_composition.cpp)
  target_link_libraries(example_dsl ${PROJECT_NAME})

  if(ament_cmake_FOUND)
    # -----------------------------------------
    # Patrol Robot Example (basic ROS2)
    # -----------------------------------------
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)

    add_executable(patrol_robot_node examples/patrol_robot_node.cpp)
    target_link_libraries(patrol_robot_node ${PROJECT_NAME})
    ament_target_dependencies(patrol_robot_node
      rclcpp
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
      tf2_ros
    )

    # -----------------------------------------
    # PX4 Drone Example (requires px4_msgs)
    # -----------------------------------------
    find_package(px4_msgs QUIET)
    if(px4_msgs_FOUND)
      message(STATUS "px4_msgs found - building PX4 drone example")
      
      add_executable(px4_drone_node examples/px4_drone_node.cpp)
      target_link_libraries(px4_drone_node ${PROJECT_NAME})
      ament_target_dependencies(px4_drone_node
        rclcpp
        std_msgs
        px4_msgs
      )
      
      install(TARGETS px4_drone_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
      )
    else()
      message(STATUS "px4_msgs not found - skipping PX4 drone example")
      message(STATUS "  To build PX4 example, clone px4_msgs to your workspace:")
      message(STATUS "    cd ~/ros2_ws/src")
      message(STATUS "    git clone https://github.com/PX4/px4_msgs.git")
      message(STATUS "    colcon build --packages-select px4_msgs")
    endif()
    
    install(TARGETS example_robot example_dsl patrol_robot_node
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
  else()
    install(TARGETS example_robot example_dsl
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
  endif()
endif()

# ==========================================
# LAUNCH FILES (ROS2)
# ==========================================
if(ament_cmake_FOUND)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY rviz/   DESTINATION share/${PROJECT_NAME}/rviz)
  install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
endif()

# ==========================================
# AMENT EXPORT (ROS2 only)
# ==========================================
if(ament_cmake_FOUND)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()