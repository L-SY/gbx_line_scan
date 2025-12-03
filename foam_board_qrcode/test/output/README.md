# 测试输出目录

此目录用于存放测试脚本生成的二维码图片。

运行测试脚本后，所有生成的二维码文件将保存在这里：

```bash
cd ~/gbx_line_scan/src/foam_board_qrcode
python3 test/test_generate_qrcodes.py
```

生成的文件包括：
- `test_board_*.png` - 各种尺寸的测试板二维码
- `batch_*.png` - 批量生产示例
- `test_production_real.png` - 真实生产场景模拟

这些文件不会被提交到Git版本控制中。

