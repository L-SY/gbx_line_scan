cmake_minimum_required(VERSION 3.8)
project(hk_light_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Qt dependencies for GUI
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Set SDK paths - try system-installed SDK first (where official MVS uses)
if(EXISTS "/opt/MVS")
    set(MVS_SDK_DIR "/opt/MVS")
    message(STATUS "Using system-installed SDK at: ${MVS_SDK_DIR}")
else()
    set(MVS_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../SDK/MVS-4.6.1_x86_64_20251217/MVS")
    message(STATUS "Using source SDK at: ${MVS_SDK_DIR}")
endif()

set(MVS_INCLUDE_DIR "${MVS_SDK_DIR}/include")
set(MVS_LIB_DIR "${MVS_SDK_DIR}/lib/64")

# Include directories
include_directories(
  ${MVS_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add light controller node executable
add_executable(hk_light_controller_node src/hk_light_controller_node.cpp)

# Set RPATH so the library can be found at runtime
set_target_properties(hk_light_controller_node PROPERTIES
  INSTALL_RPATH "${MVS_LIB_DIR}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_RPATH "${MVS_LIB_DIR}"
)

# Link directories
target_link_directories(hk_light_controller_node PRIVATE ${MVS_LIB_DIR})

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
target_link_libraries(hk_light_controller_node
  ${MVCAMERA_CONTROL_LIB}
  pthread
)

# Add dependencies
ament_target_dependencies(hk_light_controller_node
  rclcpp
  std_msgs
  std_srvs
)

# Add GUI executable
add_executable(light_controller_gui
  src/light_controller_gui_main.cpp
  src/light_controller_gui.cpp
  include/hk_light_controller/light_controller_gui.hpp
)

set_property(TARGET light_controller_gui PROPERTY AUTOMOC ON)

target_link_libraries(light_controller_gui
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(light_controller_gui
  rclcpp
  std_msgs
  std_srvs
)

# Install
install(TARGETS
  hk_light_controller_node
  light_controller_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

