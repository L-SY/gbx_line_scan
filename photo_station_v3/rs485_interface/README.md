# RS485 Interface ROS Package

This ROS2 package provides a flexible and extensible interface for RS485 devices using MODBUS RTU protocol. The package is designed to support multiple RS485 devices with a reusable architecture.

## What is MODBUS RTU?

**MODBUS RTU** (Remote Terminal Unit) is a binary serial communication protocol widely used in industrial automation. It's designed for:
- **RS485 physical layer**: Multi-drop network supporting multiple devices on one bus
- **Binary encoding**: More efficient than ASCII-based protocols
- **Master-Slave architecture**: One master device controls communication with multiple slave devices
- **CRC error checking**: Ensures data integrity

### Communication Format
- **Frame structure**: [Slave Address] [Function Code] [Data] [CRC16]
- **RTU mode**: Binary format with timing constraints (3.5 character times silence between frames)
- **Function codes**: 
  - 0x04: Read Holding Registers
  - 0x06: Write Single Register
  - 0x10: Write Multiple Registers

## What is 8N1 Configuration?

**8N1** refers to the serial port configuration:
- **8**: 8 data bits per byte
- **N**: No parity (None)
- **1**: 1 stop bit

This is the standard configuration for MODBUS RTU over RS485, providing:
- **8 data bits**: Enough for all ASCII and binary data
- **No parity**: MODBUS handles error detection via CRC16 instead
- **1 stop bit**: Standard for most serial communications

Common alternatives:
- **7E1**: 7 data bits, Even parity, 1 stop bit (used in some legacy systems)
- **8E1**: 8 data bits, Even parity, 1 stop bit
- **8N2**: 8 data bits, No parity, 2 stop bits (rare)

## Features

- **Modular Architecture**: Base classes for easy extension to new devices
- **MODBUS RTU Support**: Complete implementation of MODBUS RTU protocol over RS485
- **Thread-Safe Communication**: Mutex-protected serial communication
- **ROS2 Integration**: Native ROS2 node support with parameter configuration
- **Device Drivers**: Implemented drivers for specific devices

## Supported Devices

### SGF Series Photoelectric Distance Sensor
- Read distance measurements (in millimeters and micrometers)
- Configure working modes (high precision, standard, high speed)
- Switch mode configuration (normally open/closed)
- Detection output modes
- Analog output selection (0-5V, 4-20mA)
- Display mode configuration
- Zeroing and teaching operations

## Package Structure

```
rs485_interface/
├── include/rs485_interface/
│   ├── modbus_client.hpp      # MODBUS RTU client implementation
│   ├── device_base.hpp         # Base class for RS485 devices
│   └── distance_sensor.hpp     # Distance sensor driver
├── src/
│   ├── modbus_client.cpp
│   ├── device_base.cpp
│   ├── distance_sensor.cpp
│   └── distance_sensor_node.cpp  # ROS2 node for distance sensor
├── launch/
│   └── distance_sensor.launch.py
├── config/
│   └── distance_sensor_params.yaml
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Building

```bash
cd /home/yang/gbx_line_scan
colcon build --packages-select rs485_interface
source install/setup.bash
```

## Usage

### Running the Distance Sensor Node

1. **Check available serial devices:**
   ```bash
   ls /dev/ttyACM*
   ```

2. **Set proper permissions** (if needed):
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```
   Or add your user to the `dialout` group:
   ```bash
   sudo usermod -a -G dialout $USER
   ```

3. **Launch the node:**
   ```bash
   ros2 launch rs485_interface distance_sensor.launch.py device_path:=/dev/ttyACM0
   ```

   Or with custom parameters:
   ```bash
   ros2 launch rs485_interface distance_sensor.launch.py \
     device_path:=/dev/ttyACM0 \
     baud_rate:=115200 \
     slave_address:=1 \
     publish_rate:=10.0
   ```

### Topics

- `/distance/range` (sensor_msgs/msg/Range): Distance measurement as Range message
- `/distance/value` (std_msgs/msg/Float64): Distance value in millimeters

### Services

- `/distance/zero` (std_srvs/srv/Trigger): Perform sensor zeroing operation

### Parameters

- `device_path` (string): Path to serial device (default: "/dev/ttyACM0")
- `baud_rate` (int): Baud rate - 9600, 19200, 38400, 57600, 115200, or 256000 (default: 115200)
- `slave_address` (int): MODBUS slave address 1-255 (default: 1)
- `publish_rate` (double): Publishing rate in Hz (default: 10.0)
- `frame_id` (string): Frame ID for sensor messages (default: "distance_sensor")
- `min_range` (double): Minimum measurement range in mm (default: 0.0)
- `max_range` (double): Maximum measurement range in mm (default: 500.0)
- `field_of_view` (double): Field of view in radians (default: 0.1)
- `timeout_ms` (int): Communication timeout in milliseconds (default: 1000)

## Extending for New Devices

To add support for a new RS485 MODBUS device:

1. **Create a new device class** inheriting from `DeviceBase`:
   ```cpp
   #include "rs485_interface/device_base.hpp"
   
   class MyNewDevice : public rs485_interface::DeviceBase
   {
   public:
     MyNewDevice(std::shared_ptr<ModbusClient> modbus_client, uint8_t slave_address);
     bool initialize() override;
     std::string getDeviceType() const override;
     // Add device-specific methods
   };
   ```

2. **Implement device-specific functionality** using the MODBUS client methods:
   - `readHoldingRegisters()` - Read data from device
   - `writeSingleRegister()` - Write single parameter
   - `writeMultipleRegisters()` - Write multiple parameters

3. **Create a ROS2 node** (if needed) to publish device data

4. **Add launch and config files** for easy deployment

## Architecture

### ModbusClient
The base MODBUS RTU client handles:
- Serial port configuration (8N1, no parity, 1 stop bit)
- CRC16 calculation and verification
- Frame construction and parsing
- Thread-safe communication

### DeviceBase
Abstract base class for all RS485 devices providing:
- Common initialization interface
- Error handling
- MODBUS client access

### Device Drivers
Device-specific implementations (e.g., `DistanceSensor`) provide:
- Device-specific register mappings
- High-level API for device operations
- Parameter validation

## Invalid Measurement Handling

### Weak Reflection / Out of Range Detection

When the sensor cannot measure properly (e.g., weak reflection, out of measurement range), it returns the value `0x7FFFFFFF` (2147483647 micrometers = 2147483.647 mm), which is the 32-bit signed integer maximum value.

**Automatic Handling:**
- The driver automatically detects this invalid value
- When detected, the node publishes `NaN` (Not a Number) instead of the invalid value
- A warning message is logged (throttled to once per 5 seconds)

**Downstream Processing:**
Downstream nodes can check for invalid measurements using:
```cpp
#include <cmath>

if (std::isnan(distance_msg.data)) {
  // Handle invalid measurement
}
```

or in Python:
```python
import math

if math.isnan(distance_msg.data):
    # Handle invalid measurement
```

## Troubleshooting

### Serial Port Permissions
If you get permission denied errors:
```bash
sudo chmod 666 /dev/ttyACM0
# Or permanently add user to dialout group
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### No Response from Device
- Check that the device is powered on
- Verify the correct serial port (use `ls /dev/tty*`)
- Check baud rate matches device configuration
- Verify MODBUS slave address is correct
- Check RS485 wiring connections

### CRC Errors
- Ensure proper RS485 termination
- Check cable length and quality
- Verify baud rate settings
- Try reducing baud rate if communication is unstable

## License

MIT

## Maintainer

Yang

