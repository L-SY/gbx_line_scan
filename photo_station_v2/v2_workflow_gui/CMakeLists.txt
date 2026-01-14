cmake_minimum_required(VERSION 3.8)
project(v2_workflow_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(control_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Qt dependencies
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)  # Include current build directory for MOC files

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

# Add image viewer executable
# Note: CMAKE_AUTOMOC must be set before add_executable
# and the header file must be listed for MOC to process it
add_executable(v2_workflow_gui 
  src/v2_workflow_gui_main.cpp
  src/v2_workflow_gui_widget.cpp
  include/v2_workflow_gui/v2_workflow_gui_widget.hpp
)

# Ensure MOC runs for the header file
set_property(TARGET v2_workflow_gui PROPERTY AUTOMOC ON)

target_link_libraries(v2_workflow_gui
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(v2_workflow_gui
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
  control_msgs
)

# Install
install(TARGETS
  v2_workflow_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

