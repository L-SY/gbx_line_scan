# Photo Station V2 - Application Layer

This package provides application-specific configuration, launch files, and URDF descriptions for the Photo Station V2 pan/tilt system using the `actuator_interface` hardware interface.

## Overview

The Photo Station V2 is a 2-DOF pan/tilt mechanism powered by RM2006 motors through CAN bus communication. This package demonstrates how to create an application-specific layer on top of the generic `actuator_interface` package.

## Package Contents

```
photo_station_v2/
├── config/                                 # Application-specific configuration
│   ├── photo_station_v2_hw.yaml          # Hardware configuration for Photo Station V2
│   └── photo_station_v2_controllers.yaml # Controller configuration
├── launch/                                 # Launch files
│   ├── photo_station_v2.launch.py        # Main system launch
│   └── test_photo_station_v2.launch.py   # Test launch without RViz
├── urdf/                                   # Robot description
│   └── photo_station_v2.urdf.xacro       # URDF model for Photo Station V2
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package metadata
└── README.md                               # This file
```

## Hardware Specifications

- **DOF**: 2 (Pan + Tilt)
- **Motors**: 2x RM2006 brushless DC motors
- **Communication**: CAN bus at 1 Mbps
- **Control**: Effort interface with velocity control bridge
- **Encoders**: 14-bit absolute encoders with multi-turn tracking

### Joint Configuration
- **Joint 1 (Pan)**: Horizontal rotation, range ±180°
- **Joint 2 (Tilt)**: Vertical rotation, range ±90°

## Quick Start

### 1. Prerequisites
```bash
# Ensure actuator_interface package is built and sourced
cd your_ros2_workspace
colcon build --packages-select actuator_interface
source install/setup.bash
```

### 2. Build This Package
```bash
colcon build --packages-select photo_station_v2
source install/setup.bash
```

### 3. Hardware Setup
```bash
# Configure CAN interface
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Verify CAN connectivity
candump can0
```

### 4. Launch System
```bash
# Launch complete system with RViz
ros2 launch photo_station_v2 photo_station_v2.launch.py

# Launch for testing without RViz
ros2 launch photo_station_v2 photo_station_v2.launch.py gui:=false
```

## Control Architecture

The system uses a layered control architecture with naming convention:

```
┌─────────────────────────────────────┐
│     Application Layer               │
│  (Send velocity or effort commands) │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   ros2_control Framework            │
│  - effort_controllers (direct)      │
│  - pid_controllers (vel→effort)     │ 
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Hardware Interface                │
│  (effort interface + auto mapping)  │
│   joint1 → motor1, joint2 → motor2  │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   CAN Bus + RM2006 Motors           │
│  (torque/current control)           │
└─────────────────────────────────────┘
```

**Benefits:**
- **Clean separation**: Hardware provides pure effort interface
- **Standard PID**: Uses built-in ros2_control PID controllers
- **No custom nodes**: Everything handled within ros2_control framework
- **Auto mapping**: Simple naming convention (joint1→motor1, joint2→motor2)
- **Easy tuning**: PID parameters configurable via YAML files

## Usage

### Control Commands

The system provides effort control with velocity control bridge:

```bash
# Direct effort commands (in N⋅m) to hardware
ros2 topic pub /effort_controller1/commands std_msgs/msg/Float64MultiArray "data: [0.1]"
ros2 topic pub /effort_controller2/commands std_msgs/msg/Float64MultiArray "data: [0.05]"

# Velocity commands (in rad/s) via PID controllers (convert to effort internally)
ros2 topic pub /velocity_controller1/commands std_msgs/msg/Float64 "data: 1.0"
ros2 topic pub /velocity_controller2/commands std_msgs/msg/Float64 "data: -0.5"

# Monitor joint states
ros2 topic echo /joint_states

# Check all controllers status
ros2 control list_controllers
```

### Higher-Level Control

For position or velocity control, you can implement higher-level controllers that command effort:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SimplePositionController(Node):
    def __init__(self):
        super().__init__('simple_position_controller')
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publishers
        self.effort1_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller1/commands', 10)
        self.effort2_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller2/commands', 10)
        
        # Simple P controller gains
        self.kp1 = 2.0  # Pan joint
        self.kp2 = 1.5  # Tilt joint
        
        # Target positions (rad)
        self.target_pan = 0.0
        self.target_tilt = 0.0
        
    def joint_state_callback(self, msg):
        if len(msg.position) >= 2:
            # Simple proportional control
            pan_error = self.target_pan - msg.position[0]
            tilt_error = self.target_tilt - msg.position[1]
            
            # Calculate effort commands
            pan_effort = self.kp1 * pan_error
            tilt_effort = self.kp2 * tilt_error
            
            # Publish commands
            pan_msg = Float64MultiArray()
            pan_msg.data = [pan_effort]
            self.effort1_pub.publish(pan_msg)
            
            tilt_msg = Float64MultiArray()
            tilt_msg.data = [tilt_effort]
            self.effort2_pub.publish(tilt_msg)

def main():
    rclpy.init()
    controller = SimplePositionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration

### Hardware Configuration (`config/photo_station_v2_hw.yaml`)

Configuration is simplified with automatic joint-to-motor mapping:
- **Joint Definition**: In URDF (`joint1`, `joint2`, etc.)
- **Device Definition**: In YAML (`motor1`, `motor2`, etc.) 
- **Auto Mapping**: `joint1` ↔ `motor1`, `joint2` ↔ `motor2`

Key parameters you can modify:
- `max_effort`: Maximum motor torque (default: 10000)
- `cutoff_freq`: Velocity filter frequency (default: 100.0 Hz)
- `gear_ratio`: Mechanical gear ratio (default: 1.0)
- `direction`: Motor direction (1 or -1)

### Controller Configuration (`config/photo_station_v2_controllers.yaml`)

- `update_rate`: Control loop frequency (default: 1000 Hz)
- Controller types: Currently using `JointGroupEffortController`

## Customization for Other Applications

This package serves as a template for creating application-specific packages:

1. **Copy the package structure**:
   ```bash
   cp -r photo_station_v2 your_robot_control
   ```

2. **Update package.xml**: Change name, description, and dependencies

3. **Modify hardware configuration**: Update motor IDs, parameters, and joint definitions

4. **Adapt URDF**: Create your robot's kinematic description

5. **Customize controllers**: Choose appropriate controllers for your application

## Troubleshooting

### Common Issues

1. **CAN bus not responding**:
   ```bash
   # Check interface status
   ip link show can0
   
   # Restart interface
   sudo ip link set down can0
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set up can0
   ```

2. **Controllers not starting**:
   ```bash
   # Check hardware interface
   ros2 control list_hardware_interfaces
   
   # Check controller manager
   ros2 control list_controllers
   ```

3. **Joint states not publishing**:
   ```bash
   # Verify joint state broadcaster
   ros2 control list_controllers
   
   # Check topics
   ros2 topic list | grep joint
   ```

### Performance Tuning

- **Control Frequency**: Increase `update_rate` for better performance (up to 1000 Hz)
- **Filter Settings**: Adjust `cutoff_freq` for velocity filtering
- **Motor Parameters**: Tune `max_effort` based on load requirements
- **Velocity Control**: Tune PID parameters (`p`, `i`, `d`) for velocity controllers

## Dependencies

- `actuator_interface`: The generic hardware interface package
- `controller_manager`: ROS2 control framework
- `effort_controllers`: Effort-based control
- `joint_state_broadcaster`: Joint state publishing
- `robot_state_publisher`: Robot model publishing
- `rviz2`: Visualization (optional)

## License

MIT License - see LICENSE file for details. 