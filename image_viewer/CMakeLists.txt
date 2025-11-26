cmake_minimum_required(VERSION 3.8)
project(image_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
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
add_executable(image_viewer 
  src/image_viewer_main.cpp
  src/image_viewer_widget.cpp
  include/image_viewer/image_viewer_widget.hpp
)

# Ensure MOC runs for the header file
set_property(TARGET image_viewer PROPERTY AUTOMOC ON)

target_link_libraries(image_viewer
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(image_viewer
  rclcpp
  sensor_msgs
  cv_bridge
  control_msgs
)

# Install
install(TARGETS
  image_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

