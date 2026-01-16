#!/bin/bash

# STM32 烧录脚本（支持 ST-LINK、DAP-LINK 和 CK-LINK）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 选择构建类型
BUILD_TYPE="${1:-Debug}"
PROBE_TYPE="${2:-auto}"

BUILD_DIR="build/$BUILD_TYPE"
BIN_FILE="$BUILD_DIR/LEDtest.bin"
HEX_FILE="$BUILD_DIR/LEDtest.hex"
ELF_FILE="$BUILD_DIR/LEDtest.elf"

# 检查文件是否存在
if [ ! -f "$ELF_FILE" ]; then
    echo "错误: 未找到编译文件 $ELF_FILE"
    echo "请先运行 ./build.sh $BUILD_TYPE 进行编译"
    exit 1
fi

# 如果没有生成 bin 和 hex，生成它们
if [ ! -f "$BIN_FILE" ]; then
    echo "生成 BIN 文件..."
    arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
fi

if [ ! -f "$HEX_FILE" ]; then
    echo "生成 HEX 文件..."
    arm-none-eabi-objcopy -O ihex "$ELF_FILE" "$HEX_FILE"
fi

echo "准备烧录: $HEX_FILE"
echo "目标: STM32F103xB"

# 检查调试器是否连接
if lsusb | grep -iE "(cmsis|dap|stlink|jlink)" > /dev/null; then
    echo "检测到调试器:"
    lsusb | grep -iE "(cmsis|dap|stlink|jlink)"
else
    echo "警告: 未检测到调试器设备"
fi

# 检测调试器类型
HAS_STLINK=$(lsusb | grep -i "stlink" | wc -l)
HAS_CMSIS=$(lsusb | grep -iE "(cmsis|dap)" | wc -l)

# 优先使用 ST-LINK（如果检测到）
if [ "$HAS_STLINK" -gt 0 ]; then
    echo ""
    echo "检测到 ST-LINK 调试器，使用 ST-LINK 进行烧录..."
    
    # 优先使用 st-flash（ST-LINK 官方工具，更安全）
    if command -v st-flash &> /dev/null; then
        echo "使用 st-flash 进行烧录..."
        echo ""
        # st-flash 使用 bin 文件，更安全
        if st-flash write "$BIN_FILE" 0x8000000; then
            echo ""
            echo "烧录完成！"
            exit 0
        else
            echo ""
            echo "st-flash 烧录失败，尝试使用 openocd..."
        fi
    fi
    
    # 使用 openocd with stlink driver
    if command -v openocd &> /dev/null; then
        echo ""
        echo "使用 openocd (ST-LINK) 进行烧录..."
        
        OPENOCD_CFG=$(mktemp)
        cat > "$OPENOCD_CFG" << EOF
adapter driver stlink
transport select hla_swd
source [find target/stm32f1x.cfg]
reset_config none separate
init
reset halt
program $HEX_FILE verify
reset run
shutdown
EOF
        
        echo "使用配置文件: $OPENOCD_CFG"
        echo "开始烧录..."
        echo ""
        
        if openocd -f "$OPENOCD_CFG" || sudo openocd -f "$OPENOCD_CFG"; then
            echo ""
            echo "烧录完成！"
            rm -f "$OPENOCD_CFG"
            exit 0
        else
            rm -f "$OPENOCD_CFG"
            echo "openocd (ST-LINK) 烧录失败"
        fi
    fi
    
    echo ""
    echo "ST-LINK 烧录失败。"
    echo "建议安装 st-flash: sudo apt install stlink-tools"
    exit 1
fi

# 使用 CMSIS-DAP（如果检测到）
if [ "$HAS_CMSIS" -gt 0 ] && command -v openocd &> /dev/null; then
    echo ""
    echo "使用 openocd (CMSIS-DAP) 进行烧录..."
    
    # 尝试从 lsusb 获取 USB 设备信息
    USB_INFO=$(lsusb | grep -iE "(cmsis|dap)" | head -1)
    if [ -n "$USB_INFO" ]; then
        USB_BUS=$(echo "$USB_INFO" | awk '{print $2}')
        USB_DEV=$(echo "$USB_INFO" | awk '{print $4}' | tr -d ':')
        USB_VID=$(echo "$USB_INFO" | awk '{print $6}' | cut -d: -f1)
        USB_PID=$(echo "$USB_INFO" | awk '{print $6}' | cut -d: -f2)
        echo "检测到设备: USB $USB_BUS:$USB_DEV (VID:PID=$USB_VID:$USB_PID)"
    fi
    
    # 创建临时 openocd 配置文件
    OPENOCD_CFG=$(mktemp)
    cat > "$OPENOCD_CFG" << EOF
adapter driver cmsis-dap
transport select swd
adapter speed 1000
source [find target/stm32f1x.cfg]
reset_config none separate
init
reset halt
program $HEX_FILE verify
reset run
shutdown
EOF
    
    echo "使用配置文件: $OPENOCD_CFG"
    echo "开始烧录..."
    echo ""
    
    # 先尝试普通权限
    if openocd -f "$OPENOCD_CFG"; then
        echo ""
        echo "烧录完成！"
        rm -f "$OPENOCD_CFG"
        exit 0
    else
        echo ""
        echo "普通权限失败，尝试使用 sudo..."
        if sudo openocd -f "$OPENOCD_CFG"; then
            echo ""
            echo "烧录完成！"
            rm -f "$OPENOCD_CFG"
            exit 0
        else
            echo ""
            echo "openocd 烧录失败，尝试使用 pyocd..."
            rm -f "$OPENOCD_CFG"
            
            # 尝试使用 pyocd
            if command -v pyocd &> /dev/null; then
                echo ""
                echo "使用 pyocd 进行烧录..."
                pyocd flash --target stm32f103cb --format hex "$HEX_FILE" || {
                    echo "pyocd 烧录失败"
                    exit 1
                }
                echo ""
                echo "烧录完成！"
                exit 0
            fi
        fi
    fi
    
# 检查 pyocd（备选方案）
elif command -v pyocd &> /dev/null; then
    echo ""
    echo "使用 pyocd 进行烧录..."
    pyocd flash --target stm32f103cb --format hex "$HEX_FILE"
    echo ""
    echo "烧录完成！"
    
else
    echo "错误: 未找到烧录工具"
    echo ""
    if [ "$HAS_STLINK" -gt 0 ]; then
        echo "检测到 ST-LINK 设备，推荐安装 st-flash:"
        echo "  sudo apt install stlink-tools"
    elif [ "$HAS_CMSIS" -gt 0 ]; then
        echo "检测到 CMSIS-DAP 设备，推荐安装 openocd:"
        echo "  sudo apt install openocd"
    else
        echo "推荐安装 openocd:"
        echo "  sudo apt install openocd"
        echo ""
        echo "或安装 stlink-tools (ST-LINK):"
        echo "  sudo apt install stlink-tools"
    fi
    echo ""
    echo "或安装 pyocd:"
    echo "  pip install pyocd"
    echo ""
    echo "安装后重新运行此脚本"
    exit 1
fi
