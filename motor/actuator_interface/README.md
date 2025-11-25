# Actuator Interface - Generic CAN-based Actuator Interface for ROS2

A comprehensive and extensible ROS2 package providing hardware abstraction for CAN-based actuators, designed for seamless integration with ros2_control.

## 🎯 **Key Features**

### **Extensible Architecture**
- **Dynamic Configuration**: YAML-based motor configuration without hardcoded limits
- **Multi-Motor Support**: Unlimited motors across multiple CAN buses
- **Modular Design**: Separate hardware interface from application-specific configuration
- **ros2_control Integration**: Full compliance with ROS2 control framework

### **Supported Hardware**
- **RM Series Motors**: RM2006, RM3508, and other RM actuators
- **CAN Communication**: High-speed CAN bus communication via SocketCAN
- **Effort Control**: Pure torque/current control interface for precise motor control
- **Real-time Performance**: 1000Hz control loop capability

### **Advanced Features**
- **Multi-turn Position Tracking**: Continuous position tracking across encoder overflows
- **Configurable Parameters**: Motor-specific settings through YAML configuration
- **Thread-safe Operation**: Concurrent read/write operations
- **Extensive Diagnostics**: Comprehensive logging and monitoring

## 📦 **Package Structure**

```
actuator_interface/
├── include/actuator_interface/             # Header files
│   ├── can_manager/                        # CAN communication layer
│   │   ├── can_devices/                    # Device implementations
│   │   │   ├── can_device.h               # Base device interface
│   │   │   ├── can_actuator.h             # Actuator base class
│   │   │   └── rm_actuator.h              # RM series implementation
│   │   ├── can_interface/                  # CAN interface layer
│   │   │   ├── can_bus.h                  # CAN bus abstraction
│   │   │   └── socketcan.h                # Linux SocketCAN implementation
│   │   └── can_manager.h                   # Main CAN manager
│   ├── common/                             # Common utilities
│   │   └── lp_filter.h                     # Low-pass filter
│   └── actuator_interface.h                # Main hardware interface
├── src/                                    # Source files
│   ├── can_manager/                        # CAN implementation
│   ├── common/                             # Utilities implementation
│   ├── actuator_interface.cpp              # Hardware interface implementation
│   └── ros2_control_hw.cpp                 # ros2_control plugin
└── actuator_interface_hw.xml               # Plugin description
```

## 🚀 **Quick Start**

### **1. Dependencies**
```bash
# ROS2 Humble dependencies
sudo apt install ros-humble-hardware-interface ros-humble-controller-manager
sudo apt install ros-humble-effort-controllers ros-humble-joint-state-broadcaster
sudo apt install libyaml-cpp-dev
```

### **2. Build Package**
```bash
cd your_ros2_workspace
colcon build --packages-select actuator_interface
source install/setup.bash
```

### **3. Create Application Package**
Create a separate package for your specific application (see `photo_station_v2` example):

```bash
ros2 pkg create your_robot_control --build-type ament_cmake
```

### **4. Configure Hardware**
Create your hardware configuration (`config/your_robot_hw.yaml`):

```yaml
actuator_interface_hw:
  buses:
    - name: "can0"
      bitrate: 1000000
      priority: 95
      
  devices:
    - name: "motor1"
      bus: "can0"
      model: "RM2006"
      rx_id: 0x201
      tx_id: 0x200
      motor_index: 0
      config:
        max_effort: 10000
        cutoff_freq: 100.0
        gear_ratio: 1.0
        direction: 1

joint_config:
  joints:
    - name: "joint1"
      actuator: "motor1"
      type: "revolute"
      limits:
        position_min: -3.14159
        position_max: 3.14159
        velocity_max: 10.0
        effort_max: 1.2
```

### **5. Define URDF**
```xml
<ros2_control name="your_robot_hardware" type="system">
  <hardware>
    <plugin>actuator_interface/Ros2ControlHw</plugin>
    <param name="config_file">$(find your_robot_control)/config/your_robot_hw.yaml</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## ⚙️ **Configuration Reference**

### **Hardware Configuration**
- `buses`: List of CAN buses with settings
- `devices`: List of actuator devices with motor-specific parameters
- `joint_config`: Joint-to-actuator mapping and limits

### **Motor Parameters**
- `max_effort`: Maximum torque current command
- `cutoff_freq`: Low-pass filter frequency for velocity
- `gear_ratio`: Mechanical gear reduction ratio
- `direction`: Motor rotation direction (1 or -1)

### **CAN Communication**
- `rx_id`: CAN receive ID for the motor
- `tx_id`: CAN transmit ID for command frames
- `motor_index`: Motor position in multi-motor frames (0-3 for 0x200, 0-3 for 0x1FF)

## 🔧 **Advanced Usage**

### **Adding New Motor Types**
1. Inherit from `CanActuator` class
2. Implement motor-specific CAN protocol
3. Add device creation logic in `CanManager`
4. Configure motor parameters in YAML

### **Multiple CAN Buses**
```yaml
actuator_interface_hw:
  buses:
    - name: "can0"
      bitrate: 1000000
    - name: "can1"
      bitrate: 500000
      
  devices:
    - name: "motor1"
      bus: "can0"
      # ... configuration
    - name: "motor2"
      bus: "can1"
      # ... configuration
```

### **High-Frequency Control**
The interface supports up to 1000Hz control loops. Configure in your controller YAML:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
```

## 📊 **Performance Characteristics**

- **Control Frequency**: Up to 1000Hz
- **CAN Bandwidth**: Optimal frame packing for multiple motors
- **Latency**: <1ms end-to-end latency
- **Precision**: 8192 steps/revolution (0.0007 rad resolution)
- **Multi-turn Range**: Unlimited continuous rotation tracking

## 🛠 **Troubleshooting**

### **CAN Interface Issues**
```bash
# Check CAN interface status
ip link show can0

# Enable CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Monitor CAN traffic
candump can0
```

### **Hardware Interface Debugging**
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Check controller manager
ros2 control list_controllers
```

## 📝 **License**

MIT License - see LICENSE file for details.

## 🤝 **Contributing**

Contributions welcome! Please ensure:
- Code follows ROS2 coding standards
- New features include comprehensive tests
- Documentation is updated accordingly
- Motor-specific implementations maintain the generic interface

## 📚 **Additional Resources**

- [ROS2 Control Documentation](https://control.ros.org/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [RM Motor Specifications](https://www.robomaster.com/) 