#!/bin/bash
# 统一安装脚本 - 支持桌面自启动和系统服务两种方式

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "  Workflow GUI 自启动安装程序"
echo "========================================="
echo ""
echo "请选择安装方式："
echo "  1) 桌面自启动（登录后运行，需要GUI显示）"
echo "  2) 系统服务（开机即运行，无需登录）"
echo ""
read -p "请输入选项 (1/2): " choice
echo ""

case $choice in
    1)
        # 桌面自启动安装
        DESKTOP_FILE="${SCRIPT_DIR}/workflow_gui.desktop"
        AUTOSTART_DIR="$HOME/.config/autostart"
        TARGET_FILE="${AUTOSTART_DIR}/workflow_gui.desktop"

        if [ ! -f "$DESKTOP_FILE" ]; then
            echo "错误: workflow_gui.desktop 不存在"
            exit 1
        fi

        chmod +x "${SCRIPT_DIR}/start_workflow_gui.sh"
        mkdir -p "$AUTOSTART_DIR"
        cp "$DESKTOP_FILE" "$TARGET_FILE"

        echo "✓ 桌面自启动已安装"
        echo "  下次登录后会自动启动 workflow_gui"
        echo "  日志文件: ~/.workflow_gui.log"
        echo "  禁用: rm ${TARGET_FILE}"
        ;;
    2)
        # 系统服务安装
        if [ "$EUID" -ne 0 ]; then 
            echo "错误: 安装系统服务需要 root 权限"
            echo "请使用: sudo $0"
            exit 1
        fi

        STARTUP_SCRIPT="${SCRIPT_DIR}/startup.sh"
        TARGET_SERVICE="/etc/systemd/system/gbx_startup.service"

        if [ ! -f "$STARTUP_SCRIPT" ]; then
            echo "错误: startup.sh 不存在"
            exit 1
        fi

        chmod +x "$STARTUP_SCRIPT"

        # 修改 startup.sh 调用 start_workflow_gui.sh
        if ! grep -q "start_workflow_gui.sh" "$STARTUP_SCRIPT"; then
            echo "" >> "$STARTUP_SCRIPT"
            echo "# 启动 workflow_gui" >> "$STARTUP_SCRIPT"
            echo "bash ${SCRIPT_DIR}/start_workflow_gui.sh" >> "$STARTUP_SCRIPT"
        fi

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

        systemctl daemon-reload
        systemctl enable gbx_startup.service

        echo "✓ 系统服务已安装"
        echo "  开机后会自动启动 workflow_gui"
        echo "  查看状态: systemctl status gbx_startup.service"
        echo "  禁用: systemctl disable gbx_startup.service"
        ;;
    *)
        echo "无效选项，退出"
        exit 1
        ;;
esac

echo ""
echo "安装完成！"
echo ""
