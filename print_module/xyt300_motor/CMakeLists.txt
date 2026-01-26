cmake_minimum_required(VERSION 3.8)
project(xyt300_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Qt dependencies for GUI
find_package(Qt5 QUIET COMPONENTS Core Widgets)
if(Qt5_FOUND)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTORCC ON)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =============================================================================
# MODBUS Client Library
# =============================================================================

add_library(modbus_client_lib
  src/modbus_client.cpp
)

target_include_directories(modbus_client_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(modbus_client_lib
  rclcpp
)

# =============================================================================
# Motor Driver Library
# =============================================================================

add_library(xyt300_motor_lib
  src/xyt300_motor.cpp
)

target_include_directories(xyt300_motor_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(xyt300_motor_lib
  modbus_client_lib
)

ament_target_dependencies(xyt300_motor_lib
  rclcpp
)

# =============================================================================
# ROS2 Node
# =============================================================================

add_executable(xyt300_motor_node
  src/xyt300_motor_node.cpp
)

target_link_libraries(xyt300_motor_node
  xyt300_motor_lib
  modbus_client_lib
)

ament_target_dependencies(xyt300_motor_node
  rclcpp
  std_msgs
  std_srvs
)

# =============================================================================
# Qt GUI
# =============================================================================

if(Qt5_FOUND)
  add_executable(xyt300_motor_gui
    src/xyt300_motor_gui_main.cpp
    src/xyt300_motor_gui.cpp
    include/xyt300_motor/xyt300_motor_gui.hpp
  )

  set_property(TARGET xyt300_motor_gui PROPERTY AUTOMOC ON)

  target_link_libraries(xyt300_motor_gui
    xyt300_motor_lib
    modbus_client_lib
    Qt5::Core
    Qt5::Widgets
  )

  ament_target_dependencies(xyt300_motor_gui
    rclcpp
  )
endif()

# =============================================================================
# Install
# =============================================================================

install(TARGETS
  modbus_client_lib
  xyt300_motor_lib
  xyt300_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(Qt5_FOUND)
  install(TARGETS
    xyt300_motor_gui
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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
