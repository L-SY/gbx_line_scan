cmake_minimum_required(VERSION 3.8)
project(rs485_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# MODBUS client library
add_library(modbus_client_lib
  src/modbus_client.cpp
)

target_include_directories(modbus_client_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(modbus_client_lib
  rclcpp
)

# Device base library
add_library(device_base_lib
  src/device_base.cpp
)

target_include_directories(device_base_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(device_base_lib
  modbus_client_lib
)

ament_target_dependencies(device_base_lib
  rclcpp
)

# Distance sensor library
add_library(distance_sensor_lib
  src/distance_sensor.cpp
)

target_include_directories(distance_sensor_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(distance_sensor_lib
  device_base_lib
  modbus_client_lib
)

ament_target_dependencies(distance_sensor_lib
  rclcpp
)

# Distance sensor ROS node
add_executable(distance_sensor_node
  src/distance_sensor_node.cpp
)

target_link_libraries(distance_sensor_node
  distance_sensor_lib
  modbus_client_lib
)

ament_target_dependencies(distance_sensor_node
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
)

# Install libraries
install(TARGETS
  modbus_client_lib
  device_base_lib
  distance_sensor_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executable
install(TARGETS
  distance_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

