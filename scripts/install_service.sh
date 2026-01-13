#!/bin/bash
# 安装脚本 - 自动配置并安装 startup.service
# 用法: sudo ./install_service.sh

set -e

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_SCRIPT="${SCRIPT_DIR}/startup.sh"
SERVICE_FILE="${SCRIPT_DIR}/startup.service"
TARGET_SERVICE="/etc/systemd/system/gbx_startup.service"

echo "脚本目录: ${SCRIPT_DIR}"
echo "启动脚本: ${STARTUP_SCRIPT}"

# 检查 startup.sh 是否存在
if [ ! -f "$STARTUP_SCRIPT" ]; then
    echo "错误: startup.sh 不存在于 ${SCRIPT_DIR}"
    exit 1
fi

# 确保 startup.sh 有执行权限
chmod +x "$STARTUP_SCRIPT"

# 生成 service 文件内容（使用当前检测到的路径）
cat > "$TARGET_SERVICE" << EOF
[Unit]
Description=GBX Line Scan Startup Script
After=network.target

[Service]
Type=oneshot
ExecStart=${STARTUP_SCRIPT}
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "已创建 service 文件: ${TARGET_SERVICE}"

# 重新加载 systemd 配置
systemctl daemon-reload

# 启用服务
systemctl enable gbx_startup.service

echo ""
echo "安装完成！"
echo "可用命令:"
echo "  sudo systemctl start gbx_startup.service   # 立即启动"
echo "  sudo systemctl status gbx_startup.service  # 查看状态"
echo "  sudo systemctl disable gbx_startup.service # 禁用开机启动"
