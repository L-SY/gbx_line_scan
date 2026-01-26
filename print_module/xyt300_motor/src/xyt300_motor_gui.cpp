#include "xyt300_motor/xyt300_motor_gui.hpp"
#include "xyt300_motor/xyt300_motor.hpp"
#include "xyt300_motor/modbus_client.hpp"

#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QApplication>

// ============== ChangeAddressDialog Implementation ==============

ChangeAddressDialog::ChangeAddressDialog(uint8_t current_address, QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("修改从站地址");
  setModal(true);

  auto * layout = new QVBoxLayout(this);

  auto * form_layout = new QFormLayout();
  address_spin_ = new QSpinBox(this);
  address_spin_->setRange(1, 247);
  address_spin_->setValue(current_address);
  form_layout->addRow("新从站地址 (1-247):", address_spin_);
  layout->addLayout(form_layout);

  auto * button_box = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
    this
  );
  connect(button_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(button_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
  layout->addWidget(button_box);
}

uint8_t ChangeAddressDialog::getNewAddress() const
{
  return static_cast<uint8_t>(address_spin_->value());
}

// ============== Xyt300MotorGUI Implementation ==============

Xyt300MotorGUI::Xyt300MotorGUI(QWidget * parent)
: QMainWindow(parent),
  is_connected_(false),
  is_initialized_(false),
  scan_thread_(nullptr),
  scan_progress_(nullptr)
{
  setupUI();
  setWindowTitle("XYT300 Motor Control");
  resize(500, 600);
}

Xyt300MotorGUI::~Xyt300MotorGUI()
{
  disconnectMotor();
}

void Xyt300MotorGUI::setupUI()
{
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);

  auto * main_layout = new QVBoxLayout(central_widget_);
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);

  // Connection group
  connection_group_ = new QGroupBox("连接设置", this);
  auto * conn_layout = new QVBoxLayout(connection_group_);

  auto * device_layout = new QHBoxLayout();
  device_layout->addWidget(new QLabel("设备路径:", this));
  device_path_edit_ = new QLineEdit("/dev/ttyACM0", this);
  device_layout->addWidget(device_path_edit_);
  conn_layout->addLayout(device_layout);

  auto * baud_layout = new QHBoxLayout();
  baud_layout->addWidget(new QLabel("波特率:", this));
  baud_rate_combo_ = new QComboBox(this);
  baud_rate_combo_->addItems({"2400", "4800", "9600", "19200", "38400", "57600", "115200", "256000"});
  baud_rate_combo_->setCurrentText("115200");
  baud_layout->addWidget(baud_rate_combo_);
  conn_layout->addLayout(baud_layout);

  auto * address_layout = new QHBoxLayout();
  address_layout->addWidget(new QLabel("从站地址:", this));
  slave_address_spin_ = new QSpinBox(this);
  slave_address_spin_->setRange(1, 247);
  slave_address_spin_->setValue(1);
  address_layout->addWidget(slave_address_spin_);
  conn_layout->addLayout(address_layout);

  auto * conn_btn_layout = new QHBoxLayout();
  connect_btn_ = new QPushButton("连接", this);
  disconnect_btn_ = new QPushButton("断开", this);
  disconnect_btn_->setEnabled(false);
  init_btn_ = new QPushButton("初始化", this);
  init_btn_->setEnabled(false);
  change_address_btn_ = new QPushButton("修改从站地址", this);
  change_address_btn_->setEnabled(false);
  scan_ids_btn_ = new QPushButton("搜索设备ID", this);
  scan_ids_btn_->setEnabled(false);

  connect(connect_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onDisconnectClicked);
  connect(init_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onInitializeClicked);
  connect(change_address_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onChangeAddressClicked);
  connect(scan_ids_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onScanIdsClicked);

  conn_btn_layout->addWidget(connect_btn_);
  conn_btn_layout->addWidget(disconnect_btn_);
  conn_btn_layout->addWidget(init_btn_);
  conn_btn_layout->addWidget(change_address_btn_);
  conn_btn_layout->addWidget(scan_ids_btn_);
  conn_layout->addLayout(conn_btn_layout);

  main_layout->addWidget(connection_group_);

  // Control group
  control_group_ = new QGroupBox("电机控制", this);
  auto * ctrl_layout = new QVBoxLayout(control_group_);

  // Speed control with slider
  auto * speed_layout = new QVBoxLayout();
  auto * speed_label_layout = new QHBoxLayout();
  speed_label_layout->addWidget(new QLabel("速度 (RPM):", this));
  speed_value_label_ = new QLabel("0", this);
  speed_value_label_->setMinimumWidth(60);
  speed_value_label_->setAlignment(Qt::AlignRight);
  speed_label_layout->addWidget(speed_value_label_);
  speed_label_layout->addStretch();
  speed_layout->addLayout(speed_label_layout);
  
  speed_slider_ = new QSlider(Qt::Horizontal, this);
  speed_slider_->setRange(0, 85);
  speed_slider_->setValue(0);
  speed_slider_->setEnabled(false);
  speed_slider_->setTickPosition(QSlider::TicksBelow);
  speed_slider_->setTickInterval(17);  // 85/5 = 17, for 5 tick marks
  connect(speed_slider_, &QSlider::valueChanged, this, &Xyt300MotorGUI::onSpeedChanged);
  speed_layout->addWidget(speed_slider_);
  ctrl_layout->addLayout(speed_layout);

  // Direction control
  auto * dir_layout = new QHBoxLayout();
  forward_btn_ = new QPushButton("正转", this);
  reverse_btn_ = new QPushButton("反转", this);
  stop_btn_ = new QPushButton("停止", this);
  stop_btn_->setStyleSheet("font-weight: bold; background-color: #ff6b6b;");

  forward_btn_->setEnabled(false);
  reverse_btn_->setEnabled(false);
  stop_btn_->setEnabled(false);

  connect(forward_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onForwardClicked);
  connect(reverse_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onReverseClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &Xyt300MotorGUI::onStopClicked);

  dir_layout->addWidget(forward_btn_);
  dir_layout->addWidget(reverse_btn_);
  dir_layout->addWidget(stop_btn_);
  ctrl_layout->addLayout(dir_layout);

  // Enable/Disable removed - motor doesn't need enable/disable

  main_layout->addWidget(control_group_);

  // Status group
  status_group_ = new QGroupBox("状态", this);
  auto * status_layout = new QVBoxLayout(status_group_);

  status_label_ = new QLabel("未连接", this);
  current_speed_label_ = new QLabel("当前速度: -- RPM", this);
  error_label_ = new QLabel("", this);
  error_label_->setStyleSheet("color: #333333; font-style: italic;");

  status_layout->addWidget(status_label_);
  status_layout->addWidget(current_speed_label_);
  status_layout->addWidget(error_label_);

  main_layout->addWidget(status_group_);

  main_layout->addStretch();

  // Status bar
  status_bar_ = statusBar();
  status_bar_->showMessage("就绪");

  // Status update timer
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &Xyt300MotorGUI::onUpdateStatus);
}

bool Xyt300MotorGUI::connectMotor()
{
  if (is_connected_) {
    return true;
  }

  std::string device_path = device_path_edit_->text().toStdString();
  int baud_rate = baud_rate_combo_->currentText().toInt();

  // Convert baud rate
  xyt300_motor::ModbusClient::BaudRate modbus_baud_rate;
  switch (baud_rate) {
    case 2400: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_2400; break;
    case 4800: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_4800; break;
    case 9600: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_9600; break;
    case 19200: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_19200; break;
    case 38400: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_38400; break;
    case 57600: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_57600; break;
    case 115200: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_115200; break;
    case 256000: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_256000; break;
    default: modbus_baud_rate = xyt300_motor::ModbusClient::BaudRate::BAUD_115200; break;
  }

  // Create MODBUS client
  modbus_client_ = std::make_shared<xyt300_motor::ModbusClient>(
    device_path,
    modbus_baud_rate,
    1000
  );

  // Open serial port
  if (!modbus_client_->open()) {
    QMessageBox::critical(this, "错误", QString("无法打开串口: %1").arg(
      QString::fromStdString(modbus_client_->getLastError())));
    return false;
  }

  // Create motor driver
  // Default parameters: pulses_per_revolution=16, gear_ratio=70, max_output_rpm=85
  // These can be made configurable in the GUI if needed
  uint8_t slave_address = static_cast<uint8_t>(slave_address_spin_->value());
  motor_ = std::make_unique<xyt300_motor::Xyt300Motor>(
    modbus_client_,
    slave_address,
    16,  // pulses_per_revolution (Pv)
    70,  // gear_ratio
    85   // max_output_rpm
  );

  is_connected_ = true;
  updateConnectionStatus(true);
  status_bar_->showMessage("已连接");
  return true;
}

void Xyt300MotorGUI::disconnectMotor()
{
  if (!is_connected_) {
    return;
  }

  if (status_timer_) {
    status_timer_->stop();
  }

  motor_.reset();
  if (modbus_client_) {
    modbus_client_->close();
    modbus_client_.reset();
  }

  is_connected_ = false;
  is_initialized_ = false;
  updateConnectionStatus(false);
  status_bar_->showMessage("已断开");
}

void Xyt300MotorGUI::updateConnectionStatus(bool connected)
{
  is_connected_ = connected;

  connect_btn_->setEnabled(!connected);
  disconnect_btn_->setEnabled(connected);
  init_btn_->setEnabled(connected && !is_initialized_);
  change_address_btn_->setEnabled(connected);
  scan_ids_btn_->setEnabled(connected);

  forward_btn_->setEnabled(connected && is_initialized_);
  reverse_btn_->setEnabled(connected && is_initialized_);
  stop_btn_->setEnabled(connected);
  speed_slider_->setEnabled(connected && is_initialized_);

  if (connected) {
    status_label_->setText("已连接");
  } else {
    status_label_->setText("未连接");
    current_speed_label_->setText("当前速度: -- RPM");
    error_label_->setText("");
  }
}

void Xyt300MotorGUI::onConnectClicked()
{
  if (connectMotor()) {
    QMessageBox::information(this, "成功", "连接成功");
  }
}

void Xyt300MotorGUI::onDisconnectClicked()
{
  disconnectMotor();
}

void Xyt300MotorGUI::onInitializeClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }

  if (motor_->initialize()) {
    is_initialized_ = true;
    updateConnectionStatus(true);
    status_timer_->start(500);  // Update every 500ms to reduce UI blocking
    QMessageBox::information(this, "成功", "初始化成功");
    status_bar_->showMessage("已初始化");
  } else {
    QMessageBox::critical(this, "错误", QString("初始化失败: %1").arg(
      QString::fromStdString(motor_->getLastError())));
  }
}

void Xyt300MotorGUI::onForwardClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }

  if (motor_->setControlCommand(xyt300_motor::Xyt300Motor::ControlCommand::FORWARD)) {
    status_bar_->showMessage("正转中...", 2000);
  } else {
    QMessageBox::warning(this, "错误", QString("设置正转失败: %1").arg(
      QString::fromStdString(motor_->getLastError())));
  }
}

void Xyt300MotorGUI::onReverseClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }

  if (motor_->setControlCommand(xyt300_motor::Xyt300Motor::ControlCommand::REVERSE)) {
    status_bar_->showMessage("反转中...", 2000);
  } else {
    QMessageBox::warning(this, "错误", QString("设置反转失败: %1").arg(
      QString::fromStdString(motor_->getLastError())));
  }
}

void Xyt300MotorGUI::onStopClicked()
{
  if (!motor_) {
    return;
  }

  if (motor_->setControlCommand(xyt300_motor::Xyt300Motor::ControlCommand::STOP)) {
    status_bar_->showMessage("已停止", 2000);
  } else {
    QMessageBox::warning(this, "错误", QString("停止失败: %1").arg(
      QString::fromStdString(motor_->getLastError())));
  }
}

void Xyt300MotorGUI::onSpeedChanged(int value)
{
  // Update label immediately for responsive UI
  speed_value_label_->setText(QString::number(value));
  
  if (!motor_ || !is_initialized_) {
    return;
  }

  // Send speed command every time slider moves
  uint16_t speed_rpm = static_cast<uint16_t>(value);
  if (motor_->setSpeedRPM(speed_rpm)) {
    status_bar_->showMessage(QString("速度设置为 %1 RPM").arg(speed_rpm), 500);
  } else {
    // Don't show error dialog on every change, just update status bar
    status_bar_->showMessage(QString("设置速度失败: %1").arg(
      QString::fromStdString(motor_->getLastError())), 2000);
  }
}

void Xyt300MotorGUI::onChangeAddressClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }

  // Read current address
  uint8_t current_address = 0;
  if (!motor_->readMotorId(current_address)) {
    QMessageBox::warning(this, "错误", QString("读取当前地址失败: %1").arg(
      QString::fromStdString(motor_->getLastError())));
    return;
  }

  // Show dialog
  ChangeAddressDialog dialog(current_address, this);
  if (dialog.exec() == QDialog::Accepted) {
    uint8_t new_address = dialog.getNewAddress();
    if (new_address == current_address) {
      QMessageBox::information(this, "提示", "地址未改变");
      return;
    }

    // Confirm
    int ret = QMessageBox::question(
      this,
      "确认",
      QString("确定要将从站地址从 %1 修改为 %2 吗？\n\n注意：修改后需要重新连接。").arg(
        current_address).arg(new_address),
      QMessageBox::Yes | QMessageBox::No
    );

    if (ret == QMessageBox::Yes) {
      if (motor_->setMotorId(new_address)) {
        QMessageBox::information(
          this,
          "成功",
          QString("从站地址已修改为 %1\n请重新连接以使用新地址").arg(new_address)
        );
        disconnectMotor();
      } else {
        QMessageBox::critical(this, "错误", QString("修改地址失败: %1").arg(
          QString::fromStdString(motor_->getLastError())));
      }
    }
  }
}

void Xyt300MotorGUI::onScanIdsClicked()
{
  if (!motor_ || scan_progress_) {
    return;  // Already scanning
  }

  // Disable button during scan
  scan_ids_btn_->setEnabled(false);
  scan_ids_btn_->setText("搜索中...");
  status_bar_->showMessage("正在搜索设备（1-247）...");

  // Create progress dialog - scan only IDs 1-5
  scan_progress_ = new QProgressDialog("正在搜索设备ID（1-5）...", "取消", 0, 5, this);
  scan_progress_->setWindowTitle("搜索设备");
  scan_progress_->setWindowModality(Qt::WindowModal);
  scan_progress_->setMinimumDuration(0);
  scan_progress_->setValue(0);
  scan_progress_->show();
  QApplication::processEvents();

  // Perform scan in main thread but with progress updates
  // Scan only IDs 1-5 for faster scanning
  std::vector<uint8_t> found_ids;
  
  // Scan IDs 1-5
  for (uint8_t addr = 1; addr <= 5; ++addr) {
    // Update progress
    scan_progress_->setValue(addr);
    scan_progress_->setLabelText(QString("正在搜索设备ID %1/5...").arg(addr));
    QApplication::processEvents();  // Update UI
    
    // Check if cancelled
    if (scan_progress_->wasCanceled()) {
      break;
    }
    
    // Scan this address
    std::vector<uint8_t> batch_ids;
    motor_->scanMotorIds(batch_ids, addr, addr);
    found_ids.insert(found_ids.end(), batch_ids.begin(), batch_ids.end());
    
    // Update UI after each address
    QApplication::processEvents();
  }
  
  scan_progress_->setValue(5);
  QApplication::processEvents();
  
  // Process results
  onScanIdsFinished(found_ids, !found_ids.empty());
}

void Xyt300MotorGUI::onScanIdsFinished(const std::vector<uint8_t> & found_ids, bool success)
{
  // Clean up progress dialog
  if (scan_progress_) {
    scan_progress_->close();
    delete scan_progress_;
    scan_progress_ = nullptr;
  }

  // Re-enable button
  scan_ids_btn_->setEnabled(true);
  scan_ids_btn_->setText("搜索设备ID");

  if (!success || found_ids.empty()) {
    QMessageBox::information(
      this,
      "搜索结果",
      "未找到任何设备。\n请检查：\n1. 设备是否已连接并上电\n2. 串口设置是否正确\n3. 设备是否在总线上\n4. 从站地址是否在1-247范围内"
    );
    status_bar_->showMessage("搜索完成：未找到设备", 3000);
    return;
  }

  // Display found IDs
  QString ids_text;
  if (found_ids.size() == 1) {
    ids_text = QString("找到 1 个设备：\n从站地址: %1").arg(found_ids[0]);
  } else {
    ids_text = QString("找到 %1 个设备：\n从站地址: ").arg(found_ids.size());
    for (size_t i = 0; i < found_ids.size(); ++i) {
      if (i > 0) {
        ids_text += ", ";
      }
      ids_text += QString::number(found_ids[i]);
    }
  }

  QMessageBox::information(this, "搜索结果", ids_text);
  status_bar_->showMessage(QString("搜索完成：找到 %1 个设备").arg(found_ids.size()), 3000);
}

void Xyt300MotorGUI::onUpdateStatus()
{
  if (!motor_ || !is_initialized_) {
    return;
  }

  // Read current speed (silently fail if speed not set yet)
  uint16_t speed_rpm = 0;
  if (motor_->readCurrentSpeed(speed_rpm)) {
    current_speed_label_->setText(QString("当前速度: %1 RPM").arg(speed_rpm));
    error_label_->clear();
  }
  // Don't show error if speed read fails - it's normal if speed hasn't been set

  // Read status word (non-blocking, don't show errors)
  uint16_t status = 0;
  motor_->readStatusWord(status);
  // Update status display if needed
}
