cmake_minimum_required(VERSION 3.8)
project(transport_logic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rs485_interface REQUIRED)

# Get rs485_interface include directory
get_filename_component(rs485_interface_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../photo_station_v3/rs485_interface/include"
  ABSOLUTE
)

if(NOT EXISTS "${rs485_interface_INCLUDE_DIR}")
  message(FATAL_ERROR "Cannot find rs485_interface include directory. Tried: ${rs485_interface_INCLUDE_DIR}")
endif()

message(STATUS "Using rs485_interface include directory: ${rs485_interface_INCLUDE_DIR}")

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rs485_interface_INCLUDE_DIR}
)

# =============================================================================
# ROS2 Node
# =============================================================================

add_executable(transport_logic_node
  src/transport_logic_node.cpp
)

target_include_directories(transport_logic_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rs485_interface_INCLUDE_DIR}
)

# Get build directory for rs485_interface (for same workspace builds)
get_filename_component(rs485_interface_BUILD_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/../rs485_interface"
  ABSOLUTE
)

# Also check install directory (if available)
get_filename_component(workspace_root
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  ABSOLUTE
)
get_filename_component(rs485_interface_INSTALL_LIB_DIR
  "${workspace_root}/install/rs485_interface/lib"
  ABSOLUTE
)

# Build search paths list
set(rs485_interface_lib_search_paths ${rs485_interface_BUILD_DIR})
if(EXISTS "${rs485_interface_INSTALL_LIB_DIR}")
  list(APPEND rs485_interface_lib_search_paths ${rs485_interface_INSTALL_LIB_DIR})
endif()

target_link_directories(transport_logic_node PRIVATE ${rs485_interface_lib_search_paths})

target_link_libraries(transport_logic_node
  zd_motor_unified_lib
  lc_servo_motor_unified_lib
  rs485_device_base_unified_lib
  rs485_client_unified_lib
)

ament_target_dependencies(transport_logic_node
  rclcpp
  std_msgs
)

# =============================================================================
# Install
# =============================================================================

install(TARGETS
  transport_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
