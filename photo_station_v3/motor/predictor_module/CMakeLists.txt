cmake_minimum_required(VERSION 3.8)
project(predictor_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rs485_interface REQUIRED)

# Get rs485_interface include directory
# Use relative path for same workspace (development)
get_filename_component(rs485_interface_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../rs485_interface/include"
  ABSOLUTE
)

# Verify the directory exists
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
# Predictor Node
# =============================================================================

add_executable(predictor_node
  src/predictor_node.cpp
)

target_include_directories(predictor_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${rs485_interface_INCLUDE_DIR}
)

# For libraries in the same workspace, we need to link them
# Get the build directory for rs485_interface (for same workspace builds)
# In colcon, build directories are siblings: build/rs485_interface and build/predictor_module
get_filename_component(rs485_interface_BUILD_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/../rs485_interface"
  ABSOLUTE
)

message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "rs485_interface_BUILD_DIR: ${rs485_interface_BUILD_DIR}")

# Also check install directory (if available)
# Get workspace root to find install directory
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

message(STATUS "Searching for rs485_interface libraries in: ${rs485_interface_lib_search_paths}")

# Find the library files (check build directory first, then install)
find_library(LC_SERVO_MOTOR_MULTI_LIB
  NAMES lc_servo_motor_multi_lib
  PATHS ${rs485_interface_lib_search_paths}
  NO_DEFAULT_PATH
)

find_library(LC_SERVO_MOTOR_LIB
  NAMES lc_servo_motor_lib
  PATHS ${rs485_interface_lib_search_paths}
  NO_DEFAULT_PATH
)

find_library(RS485_DEVICE_BASE_SERVO_LIB
  NAMES rs485_device_base_servo_lib
  PATHS ${rs485_interface_lib_search_paths}
  NO_DEFAULT_PATH
)

find_library(RS485_CLIENT_SERVO_LIB
  NAMES rs485_client_servo_lib
  PATHS ${rs485_interface_lib_search_paths}
  NO_DEFAULT_PATH
)

# Check if all libraries were found
if(NOT LC_SERVO_MOTOR_MULTI_LIB)
  message(FATAL_ERROR "Cannot find lc_servo_motor_multi_lib in ${rs485_interface_BUILD_DIR}")
endif()
if(NOT LC_SERVO_MOTOR_LIB)
  message(FATAL_ERROR "Cannot find lc_servo_motor_lib in ${rs485_interface_BUILD_DIR}")
endif()
if(NOT RS485_DEVICE_BASE_SERVO_LIB)
  message(FATAL_ERROR "Cannot find rs485_device_base_servo_lib in ${rs485_interface_BUILD_DIR}")
endif()
if(NOT RS485_CLIENT_SERVO_LIB)
  message(FATAL_ERROR "Cannot find rs485_client_servo_lib in ${rs485_interface_BUILD_DIR}")
endif()

message(STATUS "Found rs485_interface libraries:")
message(STATUS "  lc_servo_motor_multi_lib: ${LC_SERVO_MOTOR_MULTI_LIB}")
message(STATUS "  lc_servo_motor_lib: ${LC_SERVO_MOTOR_LIB}")
message(STATUS "  rs485_device_base_servo_lib: ${RS485_DEVICE_BASE_SERVO_LIB}")
message(STATUS "  rs485_client_servo_lib: ${RS485_CLIENT_SERVO_LIB}")

# Link against rs485_interface libraries
target_link_libraries(predictor_node
  ${LC_SERVO_MOTOR_MULTI_LIB}
  ${LC_SERVO_MOTOR_LIB}
  ${RS485_DEVICE_BASE_SERVO_LIB}
  ${RS485_CLIENT_SERVO_LIB}
)

# Ensure rs485_interface is built before predictor_module
# This is handled automatically by colcon, but we can add explicit dependency
# Note: In ROS2, dependencies are typically handled at the package level via package.xml

ament_target_dependencies(predictor_node
  rclcpp
  std_msgs
)

# =============================================================================
# Install
# =============================================================================

# Install executables
install(TARGETS
  predictor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
