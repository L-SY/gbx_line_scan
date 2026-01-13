# Image Viewer

A Qt-based image viewer for ROS2, optimized for displaying large images from line scan cameras.

## Features

- ✅ **Handle large images**: Can display 4096x9000+ pixel images smoothly
- ✅ **Real-time zooming**: 10% to 500% zoom range
- ✅ **Auto-fit to window**: Automatically scales images to fit the window
- ✅ **Topic switching**: Dynamically switch between different image topics
- ✅ **Clean interface**: Simpler and more stable than RQT for large images

## Requirements

- ROS2 (tested with Humble/Iron)
- Qt5 development libraries
- OpenCV
- cv_bridge
- image_transport

### Install Qt5 (Ubuntu/Debian)

```bash
sudo apt-get install qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
```

## Building

```bash
cd /path/to/your/workspace
colcon build --packages-select image_viewer
source install/setup.bash
```

## Usage

### Run the image viewer (Recommended)

**Direct run (recommended for GUI applications):**

```bash
ros2 run image_viewer image_viewer
```

The viewer will start and subscribe to `/image_stitched` by default.

**Or via launch file:**

```bash
ros2 launch image_viewer image_viewer.launch.py
```

**Note:** GUI applications like this one work better when run directly with `ros2 run` as they require an interactive display environment.

### Change topic

- Use the topic dropdown/input box to select or type a different topic
- Click "Refresh" to update the list of available topics
- Press Enter after typing a topic name to subscribe

### Zoom controls

- **+ / - buttons**: Quick zoom in/out
- **Slider**: Fine-tune zoom level (10% - 500%)
- **Spin box**: Direct zoom percentage input
- **Fit button**: Auto-fit image to window size
- **100% button**: Reset to original size

## Interface

```
┌─────────────────────────────────────────────────────────┐
│ Controls                                                │
│ Topic: [dropdown/input] [Refresh]  - [+] [slider] [+] [100%] [Fit] │
├─────────────────────────────────────────────────────────┤
│                                                         │
│                    [Image Display Area]                 │
│                    (scrollable)                         │
│                                                         │
├─────────────────────────────────────────────────────────┤
│ Status: Size: 4096x9120 | Zoom: 25.0% | Frames: 19     │
└─────────────────────────────────────────────────────────┘
```

## Comparison with Other Tools

| Feature | This Viewer | RQT Image View | rviz2 |
|---------|-------------|----------------|-------|
| Large image support | ✅ Excellent | ❌ Limited | ✅ Excellent |
| Interface simplicity | ✅ Clean | ✅ Clean | ❌ Complex |
| Zoom features | ✅ Rich | ✅ Basic | ✅ Rich |
| Topic switching | ✅ Easy | ✅ Easy | ✅ Easy |
| Performance | ✅ Good | ⚠️ Medium | ✅ Good |

## Troubleshooting

### Viewer won't start

1. Ensure Qt5 is installed: `apt list --installed | grep qt5`
2. Check that workspace is built: `colcon build --packages-select image_viewer`
3. Source the environment: `source install/setup.bash`

### No image displayed

1. Check topic name (default: `/image_stitched`)
2. Verify topic is publishing: `ros2 topic list | grep image`
3. Check topic has data: `ros2 topic hz /image_stitched`

### Image too small or too large

- Use "Fit" button to auto-fit to window
- Use zoom controls to adjust manually

## License

MIT

