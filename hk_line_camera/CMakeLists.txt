cmake_minimum_required(VERSION 3.8)
project(hk_line_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Set SDK paths
set(MVS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../SDK/MVS-3.0.1_x86_64_20241128")
set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/include")
set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")

# Include directories
include_directories(
  ${MVS_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

# Add camera node executable
add_executable(hk_line_camera_node src/hk_line_camera_node.cpp)

# Set RPATH so the library can be found at runtime
set_target_properties(hk_line_camera_node PROPERTIES
  INSTALL_RPATH "${MVS_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_RPATH "${MVS_LIB_DIR}"
)

# Link directories - use target_link_directories for modern CMake
target_link_directories(hk_line_camera_node PRIVATE ${MVS_LIB_DIR})

# Find the library
find_library(MVCAMERA_CONTROL_LIB
  NAMES MvCameraControl
  PATHS ${MVS_LIB_DIR}
  NO_DEFAULT_PATH
)

if(NOT MVCAMERA_CONTROL_LIB)
  message(FATAL_ERROR "Could not find libMvCameraControl.so in ${MVS_LIB_DIR}")
endif()

message(STATUS "Found MvCameraControl library: ${MVCAMERA_CONTROL_LIB}")

# Link libraries
target_link_libraries(hk_line_camera_node
  ${MVCAMERA_CONTROL_LIB}
  ${OpenCV_LIBS}
  pthread
)

# Add dependencies
ament_target_dependencies(hk_line_camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Add image stitching node executable (doesn't need SDK libraries)
add_executable(image_stitching_node src/image_stitching_node.cpp)

# Link libraries for stitching node (only OpenCV and standard libraries)
target_link_libraries(image_stitching_node
  ${OpenCV_LIBS}
)

# Add dependencies
ament_target_dependencies(image_stitching_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

# Install
install(TARGETS
  hk_line_camera_node
  image_stitching_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
