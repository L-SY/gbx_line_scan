#!/bin/bash

# Build test script for motor_drive package
# This script tests if the package can be built successfully

echo "=== Motor Drive Package Build Test ==="
echo ""

# Navigate to workspace root
cd /home/yang/predict_ws

echo "1. Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

echo "2. Building motor_drive package..."
colcon build --packages-select motor_drive --cmake-args -DCMAKE_BUILD_TYPE=Debug

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "3. Testing package installation..."
    source install/setup.bash
    
    echo "4. Checking if package is found..."
    ros2 pkg list | grep motor_drive
    
    if [ $? -eq 0 ]; then
        echo "✅ Package installation successful!"
        echo ""
        echo "5. Listing available launch files..."
        ros2 pkg prefix motor_drive
        find install/motor_drive -name "*.launch.py" 2>/dev/null
        echo ""
        echo "=== Build test completed successfully! ==="
    else
        echo "❌ Package not found after installation"
        exit 1
    fi
    
else
    echo ""
    echo "❌ Build failed!"
    echo "Check the error messages above for details."
    exit 1
fi

echo ""
echo "Next steps:"
echo "1. Connect CAN hardware"
echo "2. Configure CAN interface: sudo ip link set can0 type can bitrate 1000000"
echo "3. Bring up CAN interface: sudo ip link set up can0"  
echo "4. Test launch: ros2 launch motor_drive test_photo_station_v2.launch.py" 