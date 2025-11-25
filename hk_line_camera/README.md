# HK Line Camera ROS2 Package

ROS2 package for Hikvision line scan camera using MVS SDK.

## Features

- Parameterized camera configuration
- Support for trigger settings (Trigger Selector, Mode, Source, Activation, Delay)
- Support for encoder settings (Selector, Source A/B, Trigger Mode, Counter Mode, etc.)
- Exposure time configuration
- Image publishing to ROS2 topics
- Real-time image acquisition via callback

## Requirements

- ROS2 (tested with Humble/Iron)
- MVS SDK 3.0.1 or later
- OpenCV
- cv_bridge
- image_transport

## Installation

1. Place the MVS SDK in the `src/SDK/` directory:
   ```
   src/
     SDK/
       MVS-3.0.1_x86_64_20241128/
   ```

2. Build the package:
   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select hk_line_camera
   source install/setup.bash
   ```

## Usage

### Basic Launch

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py
```

### With Custom Config File

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py config_file:=/path/to/your/config.yaml
```

### Parameters

All parameters can be configured via YAML file or command line:

#### Trigger Parameters
- `trigger_selector`: Trigger selector (9 = LineStart, 6 = FrameBurstStart)
- `trigger_mode`: Trigger mode (1 = On, 0 = Off)
- `trigger_source`: Trigger source (6 = EncoderModuleOut, 0 = Line0, etc.)
- `trigger_activation`: Trigger activation (0 = RisingEdge, 1 = FallingEdge)
- `trigger_delay`: Trigger delay in seconds

#### Encoder Parameters
- `encoder_selector`: Encoder selector (typically 0)
- `encoder_source_a`: Encoder source A line number
- `encoder_source_b`: Encoder source B line number
- `encoder_trigger_mode`: Encoder trigger mode (0 = AnyDirection)
- `encoder_counter_mode`: Encoder counter mode (0 = IgnoreDirection)
- `encoder_counter`: Encoder counter value
- `encoder_counter_max`: Maximum encoder counter value
- `encoder_max_reverse_counter`: Maximum reverse counter value

#### Exposure Parameters
- `exposure_time_us`: Exposure time in microseconds

#### Camera Selection
- `camera_index`: Index of camera to use (default: 0)
- `frame_id`: Frame ID for published images
- `image_topic`: Topic name for image publishing (default: "image_raw")

### Example: Set Exposure Time via Command Line

```bash
ros2 run hk_line_camera hk_line_camera_node --ros-args -p exposure_time_us:=500.0
```

## Topics

- `image_raw` (sensor_msgs/Image): Published camera images

## Troubleshooting

1. **No devices found**: Make sure the camera is connected and drivers are loaded
2. **Permission denied**: You may need to run with sudo or add your user to the appropriate groups
3. **Library not found**: Make sure the MVS SDK libraries are in the correct path

## License

MIT
