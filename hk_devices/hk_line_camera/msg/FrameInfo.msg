# 帧信息消息 / Frame Information Message
# 包含线阵相机的帧触发和行触发（编码器）信息

std_msgs/Header header

# 帧号和计数 / Frame number and counter
uint32 frame_num              # 帧号 / Frame number
uint32 frame_counter          # 总帧数 / Total frame counter
uint32 trigger_index          # 触发计数 / Trigger index

# 编码器信息（行触发相关）/ Encoder info (line trigger related)
uint32 first_line_encoder     # 首行编码器计数 / First line encoder count
uint32 last_line_encoder      # 尾行编码器计数 / Last line encoder count
uint32 encoder_diff           # 编码器差值（行数）/ Encoder difference (line count)

# IO 状态 / IO status
uint32 input_status           # IO 输入状态 / IO input status
uint32 output_status          # IO 输出状态 / IO output status

# 图像参数 / Image parameters
uint32 width                  # 图像宽度 / Image width
uint32 height                 # 图像高度 / Image height
float32 exposure_time         # 曝光时间(us) / Exposure time (us)
float32 gain                  # 增益 / Gain

# 时间戳 / Timestamp
uint32 dev_timestamp_high     # 设备时间戳高32位 / Device timestamp high 32 bits
uint32 dev_timestamp_low      # 设备时间戳低32位 / Device timestamp low 32 bits

# 丢包信息 / Lost packet info
uint32 lost_packet            # 丢包数 / Lost packet count
