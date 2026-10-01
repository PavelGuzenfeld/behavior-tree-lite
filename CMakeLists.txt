cmake_minimum_required(VERSION 3.22)
project(behavior_tree_lite VERSION 0.3.1 LANGUAGES CXX)

option(BUILD_TESTING "Build unit tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_BENCHMARKS "Build performance benchmarks" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake QUIET)

set(AMENT_CMAKE_FOUND ${ament_cmake_FOUND})

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_23)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

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

if(BUILD_TESTING)
  find_package(doctest QUIET)
  if(doctest_FOUND)
    enable_testing()
    add_executable(${PROJECT_NAME}_test
      test/test_types.cpp
      test/test_composite.cpp
      test/test_decorator.cpp
      test/test_leaf.cpp
      test/test_integration.cpp
      test/test_dsl.cpp
      test/test_blackboard.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME} doctest::doctest)
    add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)
  endif()
endif()

if(BUILD_BENCHMARKS)
  add_executable(${PROJECT_NAME}_bench test/bench_tree.cpp)
  target_link_libraries(${PROJECT_NAME}_bench ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_bench PRIVATE -O3 -DNDEBUG)
endif()

if(BUILD_EXAMPLES)
  add_executable(example_robot examples/robot_example.cpp)
  target_link_libraries(example_robot ${PROJECT_NAME})
  
  add_executable(example_dsl examples/minimal_composition.cpp)
  target_link_libraries(example_dsl ${PROJECT_NAME})

  install(TARGETS example_robot example_dsl
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(ament_cmake_FOUND)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()