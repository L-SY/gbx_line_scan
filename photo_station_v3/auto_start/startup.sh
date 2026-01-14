#!/bin/bash
# 桌面自启动脚本 - 启动 workflow_gui

# 设置日志文件
LOG_FILE="$HOME/.workflow_gui.log"
ERROR_LOG="$HOME/.workflow_gui_error.log"

# 确保基本环境变量
export USER="${USER:-$(whoami)}"
export HOME="${HOME:-$(eval echo ~$USER)}"
export PATH="${PATH}:/usr/local/bin:/usr/bin:/bin"

# 记录启动信息
{
    echo "========================================="
    echo "[$(date)] 开始启动workflow_gui"
    echo "用户: $USER"
    echo "HOME: $HOME"
    echo "工作目录: $(pwd)"
    echo "PATH: $PATH"
    echo "SHELL: $SHELL"
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
# 注意：在非交互式 shell 中，.bashrc 可能不会执行，需要强制 source
if [ -f "$HOME/.bashrc" ]; then
    # 使用 bash -i 来确保加载交互式配置，或者直接 source
    bash -c "source $HOME/.bashrc" >> "$LOG_FILE" 2>&1
    # 再次在当前 shell 中 source（因为上面的子 shell 不会影响当前环境）
    source "$HOME/.bashrc" >> "$LOG_FILE" 2>&1
    echo "[$(date)] 已加载 .bashrc" >> "$LOG_FILE"
    echo "[$(date)] ROS_DISTRO: ${ROS_DISTRO:-未设置}" >> "$LOG_FILE"
    echo "[$(date)] ROS2 路径: $(which ros2 2>/dev/null || echo '未找到')" >> "$LOG_FILE"
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
echo "[$(date)] 当前 PATH: $PATH" >> "$LOG_FILE"
echo "[$(date)] ros2 命令位置: $(which ros2)" >> "$LOG_FILE"

# 使用 bash -c 确保在正确的环境中执行
bash -c "source $HOME/.bashrc 2>/dev/null; nohup ros2 launch workflow_gui workflow_gui.launch.py >> $LOG_FILE 2>&1 &" >> "$LOG_FILE" 2>&1

# 等待一下让进程启动
sleep 2

# 检查进程是否真的启动了
LAUNCH_PID=$(pgrep -f "ros2 launch workflow_gui" | head -1)
if [ -n "$LAUNCH_PID" ]; then
    echo "[$(date)] workflow_gui已启动 (PID: $LAUNCH_PID)" >> "$LOG_FILE"
else
    echo "[$(date)] 错误: workflow_gui 启动失败，未找到进程" >> "$ERROR_LOG"
    echo "[$(date)] 错误: workflow_gui 启动失败，未找到进程" >> "$LOG_FILE"
    echo "[$(date)] 尝试直接执行 ros2 命令..." >> "$LOG_FILE"
    # 尝试直接执行看看错误信息
    bash -c "source $HOME/.bashrc 2>/dev/null; ros2 launch workflow_gui workflow_gui.launch.py" >> "$ERROR_LOG" 2>&1 &
fi

echo "=========================================" >> "$LOG_FILE"
