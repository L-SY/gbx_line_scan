# HK Line Camera ROS2 Package

ROS2 package for Hikvision line scan camera using MVS SDK.

## Features

- Parameterized camera configuration
- Support for trigger settings (Trigger Selector, Mode, Source, Activation, Delay)
- Support for encoder settings (Selector, Source A/B, Trigger Mode, Counter Mode, etc.)
- Exposure time configuration
- Image publishing to ROS2 topics
- Real-time image acquisition via callback
- **Image stitching node**: Accumulate multiple line scan images into a larger stitched image

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

### Trigger Modes

This package supports flexible trigger configuration for line scan cameras. You can use:
1. **Frame Trigger Only** (仅帧触发)
2. **Line Trigger Only** (仅行触发)
3. **Both Frame and Line Trigger** (同时使用帧触发和行触发) ⭐

#### Frame Trigger (帧触发) - FrameBurstStart
- Controls **when to start acquiring a frame**
- Typically triggered by external signal on Line0 or Line1
- Useful for synchronizing frame acquisition with external events

#### Line Trigger (行触发) - LineStart
- Controls **the timing of each line acquisition**
- Typically triggered by encoder output for motion-synchronized scanning
- Requires encoder configuration (EncoderSourceA/B)

#### Dual Trigger Mode (同时使用) ⭐ **RECOMMENDED**
When both triggers are enabled:
- **Frame Trigger (Line1)**: Determines when to start a new frame
- **Line Trigger (Encoder)**: Determines when to capture each line within the frame

This is the typical configuration for line scan cameras on moving platforms!

### Configuration Examples

#### 1. Use Dual Trigger (Frame + Line) - RECOMMENDED

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py config_file:=config/camera_params_dual_trigger.yaml
```

Configuration:
```yaml
frame_trigger_enabled: true   # Enable frame trigger on Line1
frame_trigger_source: 1       # Line1

line_trigger_enabled: true    # Enable line trigger from encoder
line_trigger_source: 6        # EncoderModuleOut

encoder_source_a: 3           # Encoder A on Line3
encoder_source_b: 0           # Encoder B on Line0
```

#### 2. Use Frame Trigger Only

```bash
ros2 launch hk_line_camera hk_line_camera.launch.py config_file:=config/camera_params_frame_trigger.yaml
```

#### 3. Use Line Trigger Only

```bash
# Modify camera_params.yaml: set frame_trigger_enabled to false
ros2 launch hk_line_camera hk_line_camera.launch.py
```

**Note**: The default configuration now uses **dual trigger mode**. To use line trigger only, set `frame_trigger_enabled: false` in the config file.

### Parameters

All parameters can be configured via YAML file or command line:

#### Frame Trigger Parameters (帧触发)
- `frame_trigger_enabled`: Enable/disable frame trigger (default: false)
- `frame_trigger_mode`: Trigger mode (1 = On, 0 = Off)
- `frame_trigger_source`: Trigger source line
  - 0 = Line0, 1 = Line1, 2 = Line2, etc.
- `frame_trigger_activation`: Trigger activation (0 = RisingEdge, 1 = FallingEdge)
- `frame_trigger_delay`: Trigger delay in seconds

#### Line Trigger Parameters (行触发)
- `line_trigger_enabled`: Enable/disable line trigger (default: true)
- `line_trigger_mode`: Trigger mode (1 = On, 0 = Off)
- `line_trigger_source`: Trigger source
  - 6 = EncoderModuleOut (typical for line scan)
  - 0 = Line0, 1 = Line1, etc.
- `line_trigger_activation`: Trigger activation (0 = RisingEdge, 1 = FallingEdge)
- `line_trigger_delay`: Trigger delay in seconds

**Note**: Both triggers can be enabled simultaneously for dual trigger mode.

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

## Image Stitching Node

The image stitching node can accumulate multiple line scan images vertically to create a larger stitched image. This is useful when you need to combine multiple frames from a line scan camera.

### Launch Stitching Node Only

```bash
ros2 launch hk_line_camera image_stitching.launch.py
```

### Launch Camera with Stitching (Default: Dual Trigger Mode)

The default configuration now uses dual trigger mode (frame + line trigger).

```bash
ros2 launch hk_line_camera camera_with_stitching.launch.py
```

This will:
- Enable **frame trigger on Line1** (controls when to start a frame)
- Enable **line trigger with encoder** (controls line-by-line acquisition)
- Start the image stitching node to combine line scan images

### Stitching Parameters

Configure stitching behavior via `config/stitching_params.yaml`:

- `input_topic`: Topic to subscribe to (default: "image_raw")
- `output_topic`: Topic to publish stitched images (default: "image_stitched")
- `frame_id`: Frame ID for published images
- `max_height`: Maximum height in pixels (0 = unlimited)
- `max_stitch_count`: Maximum number of images to stitch (0 = unlimited)
- `reset_on_max_height`: Reset and start new when max_height is reached
- `reset_on_max_count`: Reset and start new when max_stitch_count is reached
- `publish_periodically`: If true, publish at fixed rate instead of on each image
- `publish_rate`: Publishing rate in Hz (if publish_periodically is true)

### Example: Limit Stitched Image Height

```yaml
image_stitching_node:
  ros__parameters:
    max_height: 4800              # Stitch up to 4800 pixels high
    max_stitch_count: 10          # Stitch up to 10 frames
    reset_on_max_height: true     # Reset when limit reached
    reset_on_max_count: true
```

## Topics

### Camera Node
- `image_raw` (sensor_msgs/Image): Published camera images

### Stitching Node
- `image_raw` (sensor_msgs/Image): Input images (subscribed from camera)
- `image_stitched` (sensor_msgs/Image): Output stitched images

## Troubleshooting

1. **No devices found**: Make sure the camera is connected and drivers are loaded
2. **Permission denied**: You may need to run with sudo or add your user to the appropriate groups
3. **Library not found**: Make sure the MVS SDK libraries are in the correct path

## License

MIT
