cmake_minimum_required(VERSION 3.8)
project(microswitch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =============================================================================
# Serial Reader Library
# =============================================================================

add_library(serial_reader_lib
  src/serial_reader.cpp
)

target_include_directories(serial_reader_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(serial_reader_lib
  rclcpp
)

# =============================================================================
# ROS2 Node
# =============================================================================

add_executable(microswitch_node
  src/microswitch_node.cpp
)

target_link_libraries(microswitch_node
  serial_reader_lib
)

ament_target_dependencies(microswitch_node
  rclcpp
  std_msgs
)

# =============================================================================
# Install
# =============================================================================

install(TARGETS
  serial_reader_lib
  microswitch_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
