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

# Qt dependencies for GUI test
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

# MODBUS client library
add_library(modbus_client_lib
  src/distance_sensor/modbus_client.cpp
)

target_include_directories(modbus_client_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(modbus_client_lib
  rclcpp
)

# Device base library
add_library(device_base_lib
  src/common/device_base.cpp
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
  src/distance_sensor/distance_sensor.cpp
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

# RS485 client library
add_library(rs485_client_lib
  src/lc_stepper_motor/rs485_client.cpp
)

target_include_directories(rs485_client_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(rs485_client_lib
  rclcpp
)

# RS485 device base library
add_library(rs485_device_base_lib
  src/lc_stepper_motor/rs485_device_base.cpp
)

target_include_directories(rs485_device_base_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rs485_device_base_lib
  rs485_client_lib
)

ament_target_dependencies(rs485_device_base_lib
  rclcpp
)

# LC stepper motor library
add_library(lc_stepper_motor_lib
  src/lc_stepper_motor/lc_stepper_motor.cpp
)

target_include_directories(lc_stepper_motor_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lc_stepper_motor_lib
  rs485_device_base_lib
  rs485_client_lib
)

ament_target_dependencies(lc_stepper_motor_lib
  rclcpp
)

# Distance sensor ROS node
add_executable(distance_sensor_node
  src/distance_sensor/distance_sensor_node.cpp
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

# LC Stepper Motor Qt test GUI
if(Qt5_FOUND)
  add_executable(lc_stepper_motor_test_gui
    src/lc_stepper_motor/lc_stepper_motor_test_gui_main.cpp
    src/lc_stepper_motor/lc_stepper_motor_test_gui.cpp
    include/rs485_interface/lc_stepper_motor/lc_stepper_motor_test_gui.hpp
  )

  set_property(TARGET lc_stepper_motor_test_gui PROPERTY AUTOMOC ON)

  target_link_libraries(lc_stepper_motor_test_gui
    lc_stepper_motor_lib
    rs485_device_base_lib
    rs485_client_lib
    Qt5::Core
    Qt5::Widgets
  )

  ament_target_dependencies(lc_stepper_motor_test_gui
    rclcpp
  )
endif()

# RS485 client library for servo (with EVEN-1 parity support)
add_library(rs485_client_servo_lib
  src/lc_servo_motor/rs485_client.cpp
)

target_include_directories(rs485_client_servo_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(rs485_client_servo_lib
  rclcpp
)

# RS485 device base library for servo
add_library(rs485_device_base_servo_lib
  src/lc_servo_motor/rs485_device_base.cpp
)

target_include_directories(rs485_device_base_servo_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rs485_device_base_servo_lib
  rs485_client_servo_lib
)

ament_target_dependencies(rs485_device_base_servo_lib
  rclcpp
)

# LC servo motor library
add_library(lc_servo_motor_lib
  src/lc_servo_motor/lc_servo_motor.cpp
)

target_include_directories(lc_servo_motor_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lc_servo_motor_lib
  rs485_device_base_servo_lib
  rs485_client_servo_lib
)

ament_target_dependencies(lc_servo_motor_lib
  rclcpp
)

# LC Servo Motor Multi library
add_library(lc_servo_motor_multi_lib
  src/lc_servo_motor/lc_servo_motor_multi.cpp
)

target_include_directories(lc_servo_motor_multi_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lc_servo_motor_multi_lib
  lc_servo_motor_lib
  rs485_device_base_servo_lib
  rs485_client_servo_lib
)

ament_target_dependencies(lc_servo_motor_multi_lib
  rclcpp
)

# LC Servo Motor Qt test GUI
if(Qt5_FOUND)
  add_executable(lc_servo_motor_test_gui
    src/lc_servo_motor/lc_servo_motor_test_gui_main.cpp
    src/lc_servo_motor/lc_servo_motor_test_gui.cpp
    include/rs485_interface/lc_servo_motor/lc_servo_motor_test_gui.hpp
  )

  set_property(TARGET lc_servo_motor_test_gui PROPERTY AUTOMOC ON)

  target_link_libraries(lc_servo_motor_test_gui
    lc_servo_motor_lib
    rs485_device_base_servo_lib
    rs485_client_servo_lib
    Qt5::Core
    Qt5::Widgets
  )

  ament_target_dependencies(lc_servo_motor_test_gui
    rclcpp
  )

  # LC Servo Motor Multi-Motor Qt test GUI
  add_executable(lc_servo_motor_multi_test_gui
    src/lc_servo_motor/lc_servo_motor_multi_test_gui_main.cpp
    src/lc_servo_motor/lc_servo_motor_multi_test_gui.cpp
    include/rs485_interface/lc_servo_motor/lc_servo_motor_multi_test_gui.hpp
  )

  set_property(TARGET lc_servo_motor_multi_test_gui PROPERTY AUTOMOC ON)

  target_link_libraries(lc_servo_motor_multi_test_gui
    lc_servo_motor_multi_lib
    lc_servo_motor_lib
    rs485_device_base_servo_lib
    rs485_client_servo_lib
    Qt5::Core
    Qt5::Widgets
  )

  ament_target_dependencies(lc_servo_motor_multi_test_gui
    rclcpp
  )
endif()

# ZD Motor library
add_library(zd_motor_lib
  src/zd_motor/zd_motor.cpp
)

target_include_directories(zd_motor_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(zd_motor_lib
  rs485_device_base_servo_lib
  rs485_client_servo_lib
)

ament_target_dependencies(zd_motor_lib
  rclcpp
)

# ZD Motor Qt test GUI
if(Qt5_FOUND)
  add_executable(zd_motor_test_gui
    src/zd_motor/zd_motor_test_gui_main.cpp
    src/zd_motor/zd_motor_test_gui.cpp
    include/rs485_interface/zd_motor/zd_motor_test_gui.hpp
  )

  set_property(TARGET zd_motor_test_gui PROPERTY AUTOMOC ON)

  target_link_libraries(zd_motor_test_gui
    zd_motor_lib
    rs485_device_base_servo_lib
    rs485_client_servo_lib
    Qt5::Core
    Qt5::Widgets
  )

  ament_target_dependencies(zd_motor_test_gui
    rclcpp
  )
endif()

# Install libraries
install(TARGETS
  modbus_client_lib
  device_base_lib
  distance_sensor_lib
  rs485_client_lib
  rs485_device_base_lib
  lc_stepper_motor_lib
  rs485_client_servo_lib
  rs485_device_base_servo_lib
  lc_servo_motor_lib
  lc_servo_motor_multi_lib
  zd_motor_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executables
install(TARGETS
  distance_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(Qt5_FOUND)
  install(TARGETS
    lc_stepper_motor_test_gui
    lc_servo_motor_test_gui
    lc_servo_motor_multi_test_gui
    zd_motor_test_gui
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

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

