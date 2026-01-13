# 开机自启动脚本使用说明

## 文件说明

- `startup.sh`: 包含需要在开机时执行的命令
- `startup.service`: systemd 服务配置文件

## 安装步骤

### 1. 编辑启动脚本

编辑 `startup.sh` 文件，添加您需要在开机时执行的命令：

```bash
nano /home/yang/gbx_line_scan/src/scripts/startup.sh
```

### 2. 给脚本添加执行权限

```bash
chmod +x /home/yang/gbx_line_scan/src/scripts/startup.sh
```

### 3. 复制服务文件到 systemd 目录

```bash
sudo cp /home/yang/gbx_line_scan/src/scripts/startup.service /etc/systemd/system/
```

### 4. 重新加载 systemd 配置

```bash
sudo systemctl daemon-reload
```

### 5. 启用服务（开机自启动）

```bash
sudo systemctl enable startup.service
```

### 6. 启动服务（立即执行一次，可选）

```bash
sudo systemctl start startup.service
```

## 常用命令

### 查看服务状态
```bash
sudo systemctl status startup.service
```

### 查看服务日志
```bash
sudo journalctl -u startup.service -f
```

### 禁用开机自启动
```bash
sudo systemctl disable startup.service
```

### 停止服务
```bash
sudo systemctl stop startup.service
```

## 注意事项

1. **权限问题**：如果脚本需要执行需要 root 权限的命令，可以在命令前加 `sudo`，或者使用 `sudo` 运行整个脚本。

2. **日志文件**：脚本默认将日志写入 `/var/log/startup_script.log`。如果您的脚本需要写入日志，请确保有写入权限。

3. **环境变量**：systemd 服务默认不会加载用户的环境变量。如果需要，可以在 `.service` 文件中添加 `Environment` 指令。

4. **执行顺序**：服务配置中 `After=network.target` 确保在网络启动后执行。如果需要等待其他服务，可以修改此配置。

5. **安全提示**：`chmod 777` 会给所有用户完全权限，存在安全风险。建议使用更具体的权限设置，例如 `chmod 755` 或 `chmod 644`。

## 示例：在脚本中添加命令

```bash
#!/bin/bash
LOG_FILE="/var/log/startup_script.log"
echo "[$(date)] 启动脚本开始执行" >> "$LOG_FILE"

# 修改文件权限
chmod 755 /path/to/your/file

# 启动 ROS2 节点（示例）
source /opt/ros/humble/setup.bash
ros2 run your_package your_node &

# 设置环境变量
export ROS_DOMAIN_ID=0

echo "[$(date)] 启动脚本执行完成" >> "$LOG_FILE"
```
