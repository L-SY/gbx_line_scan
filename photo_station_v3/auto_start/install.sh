#!/bin/bash
# 桌面自启动安装脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESKTOP_FILE="${SCRIPT_DIR}/workflow_gui.desktop"
AUTOSTART_DIR="$HOME/.config/autostart"
TARGET_FILE="${AUTOSTART_DIR}/workflow_gui.desktop"

echo "========================================="
echo "  Workflow GUI 桌面自启动安装"
echo "========================================="
echo ""

if [ ! -f "$DESKTOP_FILE" ]; then
    echo "错误: workflow_gui.desktop 不存在"
    exit 1
fi

chmod +x "${SCRIPT_DIR}/startup.sh"
mkdir -p "$AUTOSTART_DIR"

# 复制 desktop 文件并替换 Exec 为绝对路径
sed "s|__SCRIPT_PATH__|${SCRIPT_DIR}|g" "$DESKTOP_FILE" > "$TARGET_FILE"

echo "✓ 桌面自启动已安装"
echo ""
echo "提示："
echo "  - 下次登录后会自动启动 workflow_gui"
echo "  - 日志文件: ~/.workflow_gui.log"
echo "  - 禁用自启动: rm ${TARGET_FILE}"
echo ""
