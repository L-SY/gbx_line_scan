#!/bin/bash
# 后台启动workflow_gui的脚本

# 获取脚本所在目录，然后定位到工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# 设置日志文件
LOG_FILE="$HOME/.workflow_gui.log"
echo "[$(date)] 开始启动workflow_gui" >> "$LOG_FILE"

# 等待桌面环境启动（可选，如果需要的话）
sleep 3

# Source ROS2环境（根据实际情况调整路径）
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash >> "$LOG_FILE" 2>&1
fi

# Source工作空间（如果存在install目录）
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash" >> "$LOG_FILE" 2>&1
fi

# 切换到工作空间src目录
cd "$WORKSPACE_ROOT/src" || exit 1

# 后台运行ros2 launch，输出重定向到日志文件
nohup ros2 launch workflow_gui workflow_gui.launch.py >> "$LOG_FILE" 2>&1 &

echo "[$(date)] workflow_gui已启动 (PID: $!)" >> "$LOG_FILE"
