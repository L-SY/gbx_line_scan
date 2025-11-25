# ROS1 to ROS2 Migration Notes

## Overview

This document outlines the migration of the photo_station_v2 motor control system from ROS1 Noetic to ROS2 Humble.

## Major Changes

### 1. Build System
- **ROS1**: `catkin` build system with `package.xml` format 2
- **ROS2**: `ament_cmake` build system with `package.xml` format 3
- **Changes**: Updated CMakeLists.txt and package.xml dependencies

### 2. Hardware Interface Framework
- **ROS1**: `hardware_interface::RobotHW` base class
- **ROS2**: `hardware_interface::SystemInterface` base class
- **Changes**: 
  - Complete rewrite of hardware interface
  - New lifecycle management methods
  - Updated state/command interface exports

### 3. Configuration Management
- **ROS1**: XmlRpc for parameter parsing
- **ROS2**: yaml-cpp for configuration files
- **Changes**: 
  - Replaced `XmlRpc::XmlRpcValue` with `YAML::Node`
  - Updated parameter access patterns
  - Maintained YAML file format compatibility

### 4. ROS Communication
- **ROS1**: `ros::NodeHandle`, `ROS_INFO`, etc.
- **ROS2**: `rclcpp::Node`, `RCLCPP_INFO`, etc.
- **Changes**: 
  - Updated all ROS API calls
  - Changed time handling (`ros::Time` → `rclcpp::Time`)
  - Updated logging macros

### 5. Launch System
- **ROS1**: XML-based launch files
- **ROS2**: Python-based launch files
- **Changes**: 
  - Converted all launch files to Python
  - Updated parameter passing mechanisms
  - Added proper node lifecycle management

### 6. Controller Framework
- **ROS1**: `ros_control` controllers
- **ROS2**: `ros2_control` controllers
- **Changes**: 
  - Updated controller configuration format
  - Changed controller spawning mechanism
  - Updated controller types and interfaces

## File Structure Changes

### New Files Created
```
motor_drive/
├── config/
│   ├── photo_station_v2_hw.yaml          # Hardware configuration
│   └── photo_station_v2_controllers.yaml # Controller configuration
├── include/motor_drive/
│   ├── can_manager/
│   │   ├── can_interface/
│   │   │   ├── can_bus.h
│   │   │   └── socketcan.h
│   │   ├── can_devices/
│   │   │   ├── can_device.h
│   │   │   ├── can_actuator.h
│   │   │   └── can_rm_actuator.h
│   │   └── can_manager.h
│   ├── common/
│   │   └── lp_filter.h
│   └── hardware_interface.h
├── launch/
│   ├── photo_station_v2.launch.py
│   └── test_photo_station_v2.launch.py
├── src/
│   ├── can_manager/
│   │   └── can_interface/
│   │       └── can_bus.cpp
│   └── common/
│       └── lp_filter.cpp
├── urdf/
│   └── photo_station_v2.urdf.xacro
├── photo_station_v2_hardware_interface.xml
├── README.md
└── MIGRATION_NOTES.md
```

## Key Implementation Details

### 1. CAN Device Management
- Preserved original CAN communication logic
- Updated to use ROS2 logging and time systems
- Maintained device configuration flexibility

### 2. Motor Control
- Kept RM2006 motor-specific implementation
- Updated control modes and parameter handling
- Preserved multi-turn position tracking

### 3. Configuration System
- Maintained YAML-based configuration
- Added support for different motor setups
- Preserved parameter precedence system

### 4. Real-time Performance
- Maintained 1000Hz control loop capability
- Preserved CAN bus priority settings
- Kept low-latency communication paths

## Usage Differences

### ROS1 Commands
```bash
# Launch system
roslaunch photo_station_v2 load_hw.launch
roslaunch photo_station_v2 load_controller.launch

# Control motors
rostopic pub /velocity1_controller/command std_msgs/Float64 "data: 1.0"
```

### ROS2 Commands
```bash
# Launch system
ros2 launch motor_drive photo_station_v2.launch.py

# Control motors
ros2 topic pub /velocity_controller1/commands std_msgs/msg/Float64MultiArray "data: [1.0]"
```

## Configuration Compatibility

The hardware configuration format remains largely compatible:

```yaml
# Both ROS1 and ROS2 support this format
devices:
  - name: "joint1_motor"
    bus: "can0"
    id: 0x001
    model: "RM2006"
    config:
      control_mode: "EFFORT"
      master_id: 0x201
      max_out: 10000
```

## Benefits of Migration

1. **Modern Framework**: Access to latest ROS2 features and improvements
2. **Better Performance**: Improved real-time capabilities
3. **Enhanced Lifecycle**: Better node lifecycle management
4. **Improved Tooling**: Better debugging and monitoring tools
5. **Future-Proof**: Continued support and development

## Potential Issues and Solutions

### 1. Build Dependencies
**Issue**: Some ROS1 dependencies may not be available in ROS2
**Solution**: Use ROS2 equivalents or implement missing functionality

### 2. Real-time Performance
**Issue**: Different real-time characteristics between ROS1 and ROS2
**Solution**: Tune system parameters and use appropriate QoS settings

### 3. Parameter Handling
**Issue**: Different parameter system behavior
**Solution**: Update parameter access patterns and validation

## Testing and Validation

1. **Hardware Interface**: Verify CAN communication works correctly
2. **Control Performance**: Test motor control accuracy and responsiveness
3. **Configuration**: Ensure different motor setups work as expected
4. **System Integration**: Validate complete system functionality

## Next Steps

1. Complete implementation of missing source files
2. Build and test the package
3. Validate hardware interface with actual motors
4. Performance tuning and optimization
5. Documentation and user guide updates 