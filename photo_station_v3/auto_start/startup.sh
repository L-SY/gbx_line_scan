#!/bin/bash
# 桌面自启动脚本 - 启动 workflow_gui

# 设置日志文件
LOG_FILE="$HOME/.workflow_gui.log"
ERROR_LOG="$HOME/.workflow_gui_error.log"

# 记录启动信息
{
    echo "========================================="
    echo "[$(date)] 开始启动workflow_gui"
    echo "用户: $USER"
    echo "工作目录: $(pwd)"
    echo "PATH: $PATH"
} >> "$LOG_FILE" 2>&1

# 等待桌面环境启动
sleep 5

# 修改设备权限（如果设备存在）
if [ -e /dev/ttyACM0 ]; then
    sudo chmod 777 /dev/ttyACM0 >> "$LOG_FILE" 2>&1 || {
        echo "[$(date)] 警告: chmod /dev/ttyACM0 失败" >> "$ERROR_LOG"
    }
else
    echo "[$(date)] 警告: /dev/ttyACM0 不存在" >> "$ERROR_LOG"
fi

# Source .bashrc 加载所有环境配置（包括ROS）
if [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc" >> "$LOG_FILE" 2>&1
    echo "[$(date)] 已加载 .bashrc" >> "$LOG_FILE"
else
    echo "[$(date)] 错误: .bashrc 不存在" >> "$ERROR_LOG"
fi

# 检查 ros2 命令是否可用
if ! command -v ros2 &> /dev/null; then
    echo "[$(date)] 错误: ros2 命令未找到" >> "$ERROR_LOG"
    echo "[$(date)] 错误: ros2 命令未找到" >> "$LOG_FILE"
    exit 1
fi

# 后台运行ros2 launch，输出重定向到日志文件
echo "[$(date)] 正在启动 ros2 launch workflow_gui workflow_gui.launch.py" >> "$LOG_FILE"
nohup ros2 launch workflow_gui workflow_gui.launch.py >> "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

echo "[$(date)] workflow_gui已启动 (PID: $LAUNCH_PID)" >> "$LOG_FILE"
echo "=========================================" >> "$LOG_FILE"
