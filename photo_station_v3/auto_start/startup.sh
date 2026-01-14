#!/bin/bash
# 桌面自启动脚本 - 启动 workflow_gui

# 设置日志文件
LOG_FILE="$HOME/.workflow_gui.log"
echo "[$(date)] 开始启动workflow_gui" >> "$LOG_FILE"

# 等待桌面环境启动
sleep 3

# 修改设备权限
sudo chmod 777 /dev/ttyACM0 >> "$LOG_FILE" 2>&1

# Source .bashrc 加载所有环境配置（包括ROS）
if [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc" >> "$LOG_FILE" 2>&1
fi

# 后台运行ros2 launch，输出重定向到日志文件
nohup ros2 launch workflow_gui workflow_gui.launch.py >> "$LOG_FILE" 2>&1 &

echo "[$(date)] workflow_gui已启动 (PID: $!)" >> "$LOG_FILE"
