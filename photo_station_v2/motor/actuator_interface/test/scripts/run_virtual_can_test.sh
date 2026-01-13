#!/bin/bash

echo "=========================================="
echo "虚拟CAN测试环境启动脚本"
echo "=========================================="

# 检查vcan0是否存在
if ! ip link show vcan0 &> /dev/null; then
    echo "创建虚拟CAN接口..."
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    echo "虚拟CAN接口 vcan0 已创建"
else
    echo "虚拟CAN接口 vcan0 已存在"
fi

# 显示vcan0状态
echo "vcan0 状态:"
ip link show vcan0

echo ""
echo "现在您可以:"
echo "1. 在终端1运行CAN模拟器:"
echo "   cd ~/predict_ws && python3 src/gbx_predict/actuator_interface/test/scripts/virtual_can_simulator.py"
echo ""
echo "2. 在终端2启动ROS2控制系统:"
echo "   cd ~/predict_ws && source install/setup.bash && ros2 launch actuator_interface test_virtual_can.launch.py"
echo ""
echo "3. 在终端3运行电机测试:"
echo "   cd ~/predict_ws && source install/setup.bash && python3 src/gbx_predict/actuator_interface/test/scripts/test_motors.py"
echo ""
echo "4. 监控CAN流量 (可选):"
echo "   candump vcan0"
echo ""
echo "5. 手动发送CAN测试帧 (可选):"
echo "   cansend vcan0 200#1000000000000000"
echo ""
echo "=========================================="
echo "测试文件位置:"
echo "📁 测试配置: src/gbx_predict/actuator_interface/test/config/"
echo "📁 测试脚本: src/gbx_predict/actuator_interface/test/scripts/"
echo "📁 测试启动: src/gbx_predict/actuator_interface/test/launch/"
echo "📁 测试URDF: src/gbx_predict/actuator_interface/test/urdf/"
echo "=========================================="
echo "准备开始测试!"
echo "==========================================" 