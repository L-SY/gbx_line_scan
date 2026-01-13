#!/bin/bash
# 快速安装开机自启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="startup.service"

echo "========================================="
echo "  开机自启动脚本安装程序"
echo "========================================="
echo ""

# 检查是否为 root 用户
if [ "$EUID" -ne 0 ]; then 
    echo "错误: 请使用 sudo 运行此脚本"
    echo "使用方法: sudo ./install_startup.sh"
    exit 1
fi

# 1. 给脚本添加执行权限
echo "[1/5] 设置脚本执行权限..."
chmod +x "$SCRIPT_DIR/startup.sh"
echo "  ✓ 完成"

# 2. 复制服务文件
echo "[2/5] 复制 systemd 服务文件..."
cp "$SCRIPT_DIR/$SERVICE_NAME" /etc/systemd/system/
echo "  ✓ 完成"

# 3. 重新加载 systemd
echo "[3/5] 重新加载 systemd 配置..."
systemctl daemon-reload
echo "  ✓ 完成"

# 4. 启用服务
echo "[4/5] 启用开机自启动..."
systemctl enable "$SERVICE_NAME"
echo "  ✓ 完成"

# 5. 询问是否立即启动
echo ""
read -p "[5/5] 是否立即启动服务？(y/n): " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    systemctl start "$SERVICE_NAME"
    echo "  ✓ 服务已启动"
fi

echo ""
echo "========================================="
echo "  安装完成！"
echo "========================================="
echo ""
echo "提示："
echo "  - 编辑脚本: nano $SCRIPT_DIR/startup.sh"
echo "  - 查看状态: systemctl status $SERVICE_NAME"
echo "  - 查看日志: journalctl -u $SERVICE_NAME -f"
echo "  - 禁用服务: systemctl disable $SERVICE_NAME"
echo ""
