#!/bin/bash
# 开机自启动脚本
# 此脚本会在系统启动时自动执行

# 记录日志
LOG_FILE="/var/log/startup_script.log"
echo "[$(date)] 启动脚本开始执行" >> "$LOG_FILE"

# 在这里添加您需要在开机时执行的命令
# 示例：修改文件权限
sudo chmod 777 /dev/ttyACM0

# 示例：启动某个服务
# systemctl start your-service

# 示例：设置环境变量
# export SOME_VAR="value"

# 示例：挂载文件系统
# mount /dev/sdb1 /mnt/data

# 添加您的自定义命令在这里
# ...

echo "[$(date)] 启动脚本执行完成" >> "$LOG_FILE"
