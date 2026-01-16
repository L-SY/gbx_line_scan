#!/bin/bash

# STM32 CMake 编译脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 检查工具链
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "错误: 未找到 arm-none-eabi-gcc 工具链"
    echo "请安装 ARM 工具链:"
    echo "  Ubuntu/Debian: sudo apt install gcc-arm-none-eabi"
    echo "  或从 https://developer.arm.com/downloads/-/gnu-rm 下载"
    exit 1
fi

# 检查 ninja
if ! command -v ninja &> /dev/null; then
    echo "错误: 未找到 ninja 构建工具"
    echo "请安装: sudo apt install ninja-build"
    exit 1
fi

# 检查 cmake
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到 cmake"
    echo "请安装: sudo apt install cmake"
    exit 1
fi

# 选择构建类型（Debug 或 Release）
BUILD_TYPE="${1:-Debug}"

if [ "$BUILD_TYPE" != "Debug" ] && [ "$BUILD_TYPE" != "Release" ]; then
    echo "用法: $0 [Debug|Release]"
    exit 1
fi

echo "开始编译，构建类型: $BUILD_TYPE"

# 使用 CMake Presets 配置和编译
cmake --preset "$BUILD_TYPE"
cmake --build --preset "$BUILD_TYPE"

# 生成 bin 和 hex 文件
BUILD_DIR="build/$BUILD_TYPE"
ELF_FILE="$BUILD_DIR/LEDtest.elf"

if [ -f "$ELF_FILE" ]; then
    echo "生成二进制文件..."
    arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BUILD_DIR/LEDtest.bin"
    arm-none-eabi-objcopy -O ihex "$ELF_FILE" "$BUILD_DIR/LEDtest.hex"
    
    echo ""
    echo "编译完成！"
    echo "生成的文件:"
    echo "  ELF: $ELF_FILE"
    echo "  BIN: $BUILD_DIR/LEDtest.bin"
    echo "  HEX: $BUILD_DIR/LEDtest.hex"
    echo ""
    arm-none-eabi-size "$ELF_FILE"
else
    echo "错误: 未找到编译输出文件 $ELF_FILE"
    exit 1
fi
