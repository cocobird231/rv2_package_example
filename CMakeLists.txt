cmake_minimum_required(VERSION 3.8)
project(rv2_package_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

set (SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/clock_publisher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sim_ultrasound_node.cpp
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Build
add_executable(main ${SRC_FILES})
ament_target_dependencies(main rclcpp sensor_msgs)



## Install
install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params scripts
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
