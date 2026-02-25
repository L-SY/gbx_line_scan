# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GBX Line Scan is a ROS2 workspace for an industrial line-scanning photo station system. It integrates Hikvision line scan cameras, LC servo motors, light controllers, distance sensors, and transport logic into a unified workflow.

## Build Commands

```bash
# Build entire workspace
cd ~/gbx_line_scan && colcon build

# Build a single package
cd ~/gbx_line_scan && colcon build --packages-select <package_name>

# Source the workspace after building
source ~/gbx_line_scan/install/setup.bash
```

There are no project-wide linting or test commands configured. Individual packages use standard ROS2 testing via `colcon test --packages-select <package_name>`.

## Key Launch Commands

```bash
# Full Photo Station V3 workflow (cameras + motors + lights + GUI)
ros2 launch workflow_gui workflow_gui.launch.py

# GUI only (when hardware nodes are already running)
ros2 launch workflow_gui gui_only.launch.py

# Camera with image stitching (dual trigger mode)
ros2 launch hk_line_camera camera_with_stitching.launch.py

# Distance sensor
ros2 launch rs485_interface distance_sensor.launch.py device_path:=/dev/ttyACM0

# Transport workflow
ros2 launch transport_workflow transport_workflow.launch.py
```

## Architecture

### Module Layout

- **photo_station_v3/** — Active system: workflow GUI, motor predictor (C++), RS485/MODBUS interface
- **hk_devices/** — Hardware drivers: Hikvision line camera node (C++), light controller (Python)
- **transport_module/** — Transport belt control: state machine logic, microswitch GPIO monitoring, workflow GUI
- **print_module/** — Print workflow: XYT300 motor control, TCP client to embedded board
- **photo_station_v2/** — Legacy system (actuator interface, PID controller, old GUI)
- **SDK/** — Hikvision MVS SDK (3.0.1 and 4.6.1), not built as ROS packages
- **embedded_controller/** — LED test code, marked COLCON_IGNORE

### Language Split

- **C++ (ament_cmake):** Camera driver (`hk_line_camera`), image stitching node, motor predictor (`predictor_module`), RS485/MODBUS stack (`rs485_interface`)
- **Python (ament_python):** All GUIs (`workflow_gui`, `transport_workflow`, `print_workflow`, `hk_light_controller`), transport logic, microswitch monitor

### Communication Patterns

Motors are controlled via ROS2 Float64/Bool topics (`/predictor/{front,rear}_motor/{cmd_vel,enable,velocity}`). Light sources use Int32/Bool/Float32 topics for brightness, on/off, and status telemetry. Cameras publish `sensor_msgs/Image` on namespaced topics (e.g., `/camera_front/front/image_stitched`). The image stitching node subscribes to `image_raw` and publishes `image_stitched`. The RS485 distance sensor publishes on `sensor_msgs/Range` and `std_msgs/Float64`.

### GUI Pattern

All GUIs use PyQt5 with a dark theme (background `#1a1a1a`, panels `#2a2a2a`). ROS2 callbacks run on separate threads and communicate to the Qt main thread via signals. This pattern is consistent across `workflow_gui`, `transport_workflow`, `print_workflow`, and `hk_light_controller`.

### RS485 Device Extension

New RS485 devices inherit from `DeviceBase` in `rs485_interface` and use the shared `ModbusClient` for CRC16, framing, and thread-safe serial I/O. See `distance_sensor.hpp/cpp` as a reference implementation.

### Camera Trigger Modes

The line camera supports three trigger configurations: frame-only, line-only, and dual (frame + line) which is the recommended mode. Frame triggers control when to start acquisition; line triggers (from encoder) control per-line timing. Config YAML files in `hk_line_camera/config/` select the mode.

## Hardware Dependencies

- Serial devices at `/dev/ttyACM*` (motors, RS485 sensors) — user must be in `dialout` group
- Hikvision MVS SDK libraries from `src/SDK/`
- Light controller communicates over TCP (default IP: `192.168.3.101`)
