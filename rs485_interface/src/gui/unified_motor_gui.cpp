#include "rs485_interface/gui/unified_motor_gui.hpp"
#include "rs485_interface/common/rs485_client.hpp"
#include "rs485_interface/common/rs485_device_base.hpp"
#include "rs485_interface/motor/zd_motor/zd_motor_new.hpp"
#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_new.hpp"
#include "rs485_interface/motor/lc_stepper_motor/lc_stepper_motor_new.hpp"

#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QInputDialog>

// ============== SingleMotorWidget Implementation ==============

SingleMotorWidget::SingleMotorWidget(const MotorConfig & config, QWidget * parent)
: QWidget(parent),
  config_(config),
  is_connected_(false),
  is_busy_(false)
{
  setupUI();
}

SingleMotorWidget::~SingleMotorWidget()
{
  disconnectMotor();
}

void SingleMotorWidget::setupUI()
{
  auto * main_layout = new QVBoxLayout(this);
  main_layout->setSpacing(10);
  main_layout->setContentsMargins(10, 10, 10, 10);
  
  // Connection info
  auto * info_group = new QGroupBox("Motor Info", this);
  auto * info_layout = new QFormLayout(info_group);
  
  QString type_str;
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR: type_str = "ZD Motor"; break;
    case MotorConfig::MotorType::LC_SERVO: type_str = "LC Servo"; break;
    case MotorConfig::MotorType::LC_STEPPER: type_str = "LC Stepper"; break;
  }
  
  info_layout->addRow("Name:", new QLabel(config_.name));
  info_layout->addRow("Type:", new QLabel(type_str));
  info_layout->addRow("Port:", new QLabel(config_.port));
  info_layout->addRow("Address:", new QLabel(QString::number(config_.slave_address)));
  main_layout->addWidget(info_group);
  
  // Connection controls
  auto * conn_group = new QGroupBox("Connection", this);
  auto * conn_layout = new QHBoxLayout(conn_group);
  
  connect_btn_ = new QPushButton("Connect", this);
  disconnect_btn_ = new QPushButton("Disconnect", this);
  disconnect_btn_->setEnabled(false);
  
  connect(connect_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onDisconnectClicked);
  
  conn_layout->addWidget(connect_btn_);
  conn_layout->addWidget(disconnect_btn_);
  main_layout->addWidget(conn_group);
  
  // Motor-specific controls
  auto * control_group = new QGroupBox("Control", this);
  auto * control_layout = new QVBoxLayout(control_group);
  
  // Common controls
  init_btn_ = new QPushButton("Initialize", this);
  init_btn_->setEnabled(false);
  connect(init_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onInitializeClicked);
  control_layout->addWidget(init_btn_);
  
  // Speed control (common for all motor types)
  auto * speed_layout = new QHBoxLayout();
  speed_layout->addWidget(new QLabel("Speed (RPM):"));
  speed_spin_ = new QDoubleSpinBox(this);
  speed_spin_->setRange(0, 10000);
  speed_spin_->setValue(100);
  speed_spin_->setEnabled(false);
  speed_layout->addWidget(speed_spin_);
  
  current_speed_label_ = new QLabel("Current: --", this);
  speed_layout->addWidget(current_speed_label_);
  control_layout->addLayout(speed_layout);

  // Different controls based on motor type
  if (config_.type == MotorConfig::MotorType::LC_STEPPER) {
    // Stepper: Position control mode (same behavior as old GUI)
    // Position spinbox - sends command immediately on change
    auto * pos_layout = new QHBoxLayout();
    pos_layout->addWidget(new QLabel("Position:"));
    position_spin_ = new QDoubleSpinBox(this);
    position_spin_->setRange(-2147483647.0, 2147483647.0);
    position_spin_->setValue(0);
    position_spin_->setDecimals(2);
    position_spin_->setEnabled(false);
    connect(position_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SingleMotorWidget::onPositionChanged);
    pos_layout->addWidget(position_spin_);
    control_layout->addLayout(pos_layout);
    
    // Speed spinbox - also sends immediately on change
    connect(speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SingleMotorWidget::onSpeedChanged);
    
    // Start and Stop buttons (same as old GUI)
    auto * stepper_btn_layout = new QHBoxLayout();
    start_motor_btn_ = new QPushButton("Start", this);
    start_motor_btn_->setEnabled(false);
    connect(start_motor_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onStartClicked);
    stepper_btn_layout->addWidget(start_motor_btn_);
    
    stop_btn_ = new QPushButton("Stop", this);
    stop_btn_->setStyleSheet("font-weight: bold;");
    stop_btn_->setEnabled(false);
    connect(stop_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onStopClicked);
    stepper_btn_layout->addWidget(stop_btn_);
    control_layout->addLayout(stepper_btn_layout);
    
    // Hide unused controls for stepper
    forward_btn_ = nullptr;
    reverse_btn_ = nullptr;
    set_speed_btn_ = nullptr;
  } else {
    // Servo/ZD: Direction control mode
    position_spin_ = nullptr;
    start_motor_btn_ = nullptr;
    set_speed_btn_ = nullptr;
    
    auto * dir_layout = new QHBoxLayout();
    forward_btn_ = new QPushButton("Forward", this);
    reverse_btn_ = new QPushButton("Reverse", this);
    stop_btn_ = new QPushButton("STOP", this);
    stop_btn_->setStyleSheet("font-weight: bold;");
    
    forward_btn_->setEnabled(false);
    reverse_btn_->setEnabled(false);
    stop_btn_->setEnabled(false);
    
    connect(forward_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onForwardClicked);
    connect(reverse_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onReverseClicked);
    connect(stop_btn_, &QPushButton::clicked, this, &SingleMotorWidget::onStopClicked);
    
    // For servo/ZD, speed change is immediate
    connect(speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &SingleMotorWidget::onSpeedChanged);
    
    dir_layout->addWidget(forward_btn_);
    dir_layout->addWidget(reverse_btn_);
    dir_layout->addWidget(stop_btn_);
    control_layout->addLayout(dir_layout);
  }
  
  main_layout->addWidget(control_group);
  
  // Status
  auto * status_group = new QGroupBox("Status", this);
  auto * status_layout = new QVBoxLayout(status_group);
  
  status_label_ = new QLabel("Disconnected", this);
  error_label_ = new QLabel("", this);
  error_label_->setStyleSheet("color: #333333; font-style: italic;");
  
  status_layout->addWidget(status_label_);
  status_layout->addWidget(error_label_);
  main_layout->addWidget(status_group);
  
  main_layout->addStretch();
  
  // Status update timer (only for motors that support reading status)
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &SingleMotorWidget::onUpdateStatus);
  
  // Stepper motors don't support real-time status reading, so no timer needed
  // For other motors, timer will be started on connect
}

bool SingleMotorWidget::connectMotor()
{
  if (is_connected_) {
    return true;
  }
  
  // Determine baud rate and parity
  rs485_interface::RS485Client::BaudRate baud;
  switch (config_.baud_rate) {
    case 2400: baud = rs485_interface::RS485Client::BaudRate::BAUD_2400; break;
    case 4800: baud = rs485_interface::RS485Client::BaudRate::BAUD_4800; break;
    case 9600: baud = rs485_interface::RS485Client::BaudRate::BAUD_9600; break;
    case 19200: baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
    case 38400: baud = rs485_interface::RS485Client::BaudRate::BAUD_38400; break;
    case 57600: baud = rs485_interface::RS485Client::BaudRate::BAUD_57600; break;
    case 115200: baud = rs485_interface::RS485Client::BaudRate::BAUD_115200; break;
    case 256000: baud = rs485_interface::RS485Client::BaudRate::BAUD_256000; break;
    default: baud = rs485_interface::RS485Client::BaudRate::BAUD_19200; break;
  }
  
  rs485_interface::RS485Client::Parity parity;
  switch (config_.parity) {
    case 0: parity = rs485_interface::RS485Client::Parity::NONE; break;
    case 1: parity = rs485_interface::RS485Client::Parity::EVEN; break;
    case 2: parity = rs485_interface::RS485Client::Parity::ODD; break;
    default: parity = rs485_interface::RS485Client::Parity::NONE; break;
  }
  
  // Create RS485 client
  rs485_client_ = std::make_shared<rs485_interface::RS485Client>(
    config_.port.toStdString(), baud, parity);
  
  if (!rs485_client_->open()) {
    error_label_->setText(QString::fromStdString(rs485_client_->getLastError()));
    emit errorMessage(QString::fromStdString(rs485_client_->getLastError()));
    return false;
  }
  
  // Create motor instance based on type
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      zd_motor_ = std::make_shared<rs485_interface::ZdMotor>(
        rs485_client_, config_.slave_address);
      break;
    case MotorConfig::MotorType::LC_SERVO:
      servo_motor_ = std::make_shared<rs485_interface::LcServoMotor>(
        rs485_client_, config_.slave_address);
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      stepper_motor_ = std::make_shared<rs485_interface::LcStepperMotor>(
        rs485_client_, config_.slave_address);
      break;
  }
  
  is_connected_ = true;
  updateConnectionStatus(true);
  emit connectionChanged(true);
  
  // Start status timer (with longer interval to reduce bus conflicts)
  status_timer_->start(1000);
  
  return true;
}

void SingleMotorWidget::disconnectMotor()
{
  status_timer_->stop();
  
  zd_motor_.reset();
  servo_motor_.reset();
  stepper_motor_.reset();
  
  if (rs485_client_) {
    rs485_client_->close();
    rs485_client_.reset();
  }
  
  is_connected_ = false;
  updateConnectionStatus(false);
  emit connectionChanged(false);
}

std::shared_ptr<rs485_interface::RS485DeviceBase> SingleMotorWidget::getDevice() const
{
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      return zd_motor_;
    case MotorConfig::MotorType::LC_SERVO:
      return servo_motor_;
    case MotorConfig::MotorType::LC_STEPPER:
      return stepper_motor_;
  }
  return nullptr;
}

void SingleMotorWidget::updateConnectionStatus(bool connected)
{
  connect_btn_->setEnabled(!connected);
  disconnect_btn_->setEnabled(connected);
  init_btn_->setEnabled(connected);
  speed_spin_->setEnabled(connected);
  stop_btn_->setEnabled(connected);
  
  // Enable motor-type specific controls
  if (config_.type == MotorConfig::MotorType::LC_STEPPER) {
    if (position_spin_) position_spin_->setEnabled(connected);
    if (start_motor_btn_) start_motor_btn_->setEnabled(connected);
  } else {
    if (forward_btn_) forward_btn_->setEnabled(connected);
    if (reverse_btn_) reverse_btn_->setEnabled(connected);
  }
  
  status_label_->setText(connected ? "Connected" : "Disconnected");
  if (connected) {
    status_label_->setStyleSheet("font-weight: bold;");
  } else {
    status_label_->setStyleSheet("color: #666666;");
    current_speed_label_->setText("Current: --");
  }
}

void SingleMotorWidget::onConnectClicked()
{
  connectMotor();
}

void SingleMotorWidget::onDisconnectClicked()
{
  disconnectMotor();
}

void SingleMotorWidget::onInitializeClicked()
{
  if (is_busy_) return;
  is_busy_ = true;
  
  bool success = false;
  
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      if (zd_motor_) success = zd_motor_->initialize();
      break;
    case MotorConfig::MotorType::LC_SERVO:
      if (servo_motor_) success = servo_motor_->initialize();
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      if (stepper_motor_) success = stepper_motor_->initialize();
      break;
  }
  
  if (success) {
    emit statusMessage(config_.name + ": Initialized");
  } else {
    auto device = getDevice();
    if (device) {
      error_label_->setText(QString::fromStdString(device->getLastError()));
    }
  }
  
  is_busy_ = false;
}

void SingleMotorWidget::onForwardClicked()
{
  if (is_busy_) return;
  is_busy_ = true;
  
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      if (zd_motor_) zd_motor_->setControlCommand(
        rs485_interface::ZdMotor::ControlCommand::FORWARD);
      break;
    case MotorConfig::MotorType::LC_SERVO:
      if (servo_motor_) {
        servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE);
        servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::FORWARD);
      }
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      if (stepper_motor_) stepper_motor_->setState(
        rs485_interface::LcStepperMotor::MotorState::START);
      break;
  }
  
  is_busy_ = false;
}

void SingleMotorWidget::onReverseClicked()
{
  if (is_busy_) return;
  is_busy_ = true;
  
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      if (zd_motor_) zd_motor_->setControlCommand(
        rs485_interface::ZdMotor::ControlCommand::REVERSE);
      break;
    case MotorConfig::MotorType::LC_SERVO:
      if (servo_motor_) {
        servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE);
        servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::REVERSE);
      }
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      if (stepper_motor_) stepper_motor_->setState(
        rs485_interface::LcStepperMotor::MotorState::START);
      break;
  }
  
  is_busy_ = false;
}

void SingleMotorWidget::onStopClicked()
{
  stopMotor();
}

void SingleMotorWidget::stopMotor()
{
  // Stop should always work, even if busy (emergency stop)
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      if (zd_motor_) zd_motor_->setControlCommand(
        rs485_interface::ZdMotor::ControlCommand::STOP);
      break;
    case MotorConfig::MotorType::LC_SERVO:
      if (servo_motor_) {
        servo_motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
        servo_motor_->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE);
      }
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      if (stepper_motor_) stepper_motor_->setState(
        rs485_interface::LcStepperMotor::MotorState::STOP);
      break;
  }
  is_busy_ = false;  // Reset busy flag after stop
}

void SingleMotorWidget::onSpeedChanged(double value)
{
  if (!is_connected_ || is_busy_) return;
  
  is_busy_ = true;
  
  // All motor types: send speed command immediately
  switch (config_.type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      if (zd_motor_) zd_motor_->setSpeedRPM(static_cast<uint16_t>(value));
      break;
    case MotorConfig::MotorType::LC_SERVO:
      if (servo_motor_) servo_motor_->setSpeedRPM(value);
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      if (stepper_motor_) {
        if (stepper_motor_->setSpeed(static_cast<float>(value))) {
          error_label_->clear();
        } else {
          error_label_->setText(QString::fromStdString(stepper_motor_->getLastError()));
        }
      }
      break;
  }
  
  is_busy_ = false;
}

void SingleMotorWidget::onStartClicked()
{
  if (config_.type != MotorConfig::MotorType::LC_STEPPER || !stepper_motor_ || is_busy_) {
    return;
  }
  
  is_busy_ = true;
  if (stepper_motor_->setState(rs485_interface::LcStepperMotor::MotorState::START)) {
    error_label_->clear();
    emit statusMessage(config_.name + ": Motor started");
  } else {
    error_label_->setText(QString::fromStdString(stepper_motor_->getLastError()));
  }
  is_busy_ = false;
}

void SingleMotorWidget::onPositionChanged(double value)
{
  if (config_.type != MotorConfig::MotorType::LC_STEPPER || !stepper_motor_ || !is_connected_ || is_busy_) {
    return;
  }
  
  is_busy_ = true;
  // Send position command immediately (same as old GUI)
  if (stepper_motor_->setPosition(static_cast<float>(value))) {
    error_label_->clear();
  } else {
    error_label_->setText(QString::fromStdString(stepper_motor_->getLastError()));
  }
  is_busy_ = false;
}

void SingleMotorWidget::onUpdateStatus()
{
  // Skip if not connected, busy, or widget not visible
  if (!is_connected_ || is_busy_ || !isVisible()) {
    return;
  }
  
  is_busy_ = true;
  
  // Only poll status for supported motor types
  if (config_.type == MotorConfig::MotorType::LC_SERVO && servo_motor_) {
    double speed;
    if (servo_motor_->readCurrentSpeed(speed)) {
      current_speed_label_->setText(QString("Current: %1 RPM").arg(speed, 0, 'f', 1));
    }
  }
  else if (config_.type == MotorConfig::MotorType::ZD_MOTOR && zd_motor_) {
    uint16_t status1;
    if (zd_motor_->readStatusWord1(status1)) {
      current_speed_label_->setText(QString("Status: 0x%1").arg(status1, 4, 16, QChar('0')));
    }
  }
  // Stepper motors: no status polling
  
  is_busy_ = false;
}

// ============== UnifiedMotorGUI Implementation ==============

UnifiedMotorGUI::UnifiedMotorGUI(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("RS485 Motor Control Panel");
  setMinimumSize(950, 650);
  resize(1000, 700);
  
  setupMenuBar();
  setupUI();
  
  // Status bar
  status_label_ = new QLabel("Ready", this);
  statusBar()->addWidget(status_label_);
}

UnifiedMotorGUI::~UnifiedMotorGUI()
{
  // Disconnect all motors
  for (auto & pair : motor_widgets_) {
    if (pair.second) {
      pair.second->disconnectMotor();
    }
  }
}

void UnifiedMotorGUI::setupMenuBar()
{
  auto * file_menu = menuBar()->addMenu("&File");
  
  auto * add_action = file_menu->addAction("&Add Motor...");
  connect(add_action, &QAction::triggered, this, &UnifiedMotorGUI::onAddMotorClicked);
  
  file_menu->addSeparator();
  
  auto * quit_action = file_menu->addAction("&Quit");
  connect(quit_action, &QAction::triggered, this, &QMainWindow::close);
}

void UnifiedMotorGUI::setupUI()
{
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  auto * main_layout = new QVBoxLayout(central_widget_);
  
  main_splitter_ = new QSplitter(Qt::Horizontal, this);
  
  // Left panel - Motor list
  auto * left_widget = new QWidget(this);
  auto * left_layout = new QVBoxLayout(left_widget);
  
  motor_list_group_ = new QGroupBox("Motors", left_widget);
  auto * list_layout = new QVBoxLayout(motor_list_group_);
  
  motor_list_ = new QListWidget(this);
  connect(motor_list_, &QListWidget::currentRowChanged,
          this, &UnifiedMotorGUI::onMotorSelected);
  list_layout->addWidget(motor_list_);
  
  auto * btn_layout = new QHBoxLayout();
  add_motor_btn_ = new QPushButton("Add", this);
  remove_motor_btn_ = new QPushButton("Remove", this);
  connect(add_motor_btn_, &QPushButton::clicked, this, &UnifiedMotorGUI::onAddMotorClicked);
  connect(remove_motor_btn_, &QPushButton::clicked, this, &UnifiedMotorGUI::onRemoveMotorClicked);
  btn_layout->addWidget(add_motor_btn_);
  btn_layout->addWidget(remove_motor_btn_);
  list_layout->addLayout(btn_layout);
  
  left_layout->addWidget(motor_list_group_);
  left_widget->setMaximumWidth(250);
  
  main_splitter_->addWidget(left_widget);
  
  // Right panel - Motor control tabs
  motor_tabs_ = new QTabWidget(this);
  motor_tabs_->setTabsClosable(false);
  main_splitter_->addWidget(motor_tabs_);
  
  main_layout->addWidget(main_splitter_, 1);
  
  // Bottom panel - Global controls
  global_control_group_ = new QGroupBox("Global Controls", this);
  auto * global_layout = new QHBoxLayout(global_control_group_);
  
  connect_all_btn_ = new QPushButton("Connect All", this);
  disconnect_all_btn_ = new QPushButton("Disconnect All", this);
  stop_all_btn_ = new QPushButton("STOP ALL", this);
  stop_all_btn_->setStyleSheet("font-weight: bold; padding: 10px;");
  
  connect(connect_all_btn_, &QPushButton::clicked, this, &UnifiedMotorGUI::onConnectAllClicked);
  connect(disconnect_all_btn_, &QPushButton::clicked, this, &UnifiedMotorGUI::onDisconnectAllClicked);
  connect(stop_all_btn_, &QPushButton::clicked, this, &UnifiedMotorGUI::onStopAllClicked);
  
  global_layout->addWidget(connect_all_btn_);
  global_layout->addWidget(disconnect_all_btn_);
  global_layout->addStretch();
  global_layout->addWidget(stop_all_btn_);
  
  main_layout->addWidget(global_control_group_);
}

void UnifiedMotorGUI::onAddMotorClicked()
{
  AddMotorDialog dialog(this);
  if (dialog.exec() == QDialog::Accepted) {
    MotorConfig config = dialog.getConfig();
    addMotor(config);
  }
}

void UnifiedMotorGUI::onRemoveMotorClicked()
{
  auto * current = motor_list_->currentItem();
  if (current) {
    removeMotor(current->text());
  }
}

void UnifiedMotorGUI::onConnectAllClicked()
{
  for (auto & pair : motor_widgets_) {
    if (pair.second && !pair.second->isConnected()) {
      pair.second->connectMotor();
    }
  }
}

void UnifiedMotorGUI::onDisconnectAllClicked()
{
  for (auto & pair : motor_widgets_) {
    if (pair.second && pair.second->isConnected()) {
      pair.second->disconnectMotor();
    }
  }
}

void UnifiedMotorGUI::onStopAllClicked()
{
  for (auto & pair : motor_widgets_) {
    if (pair.second && pair.second->isConnected()) {
      pair.second->stopMotor();
    }
  }
  logMessage("Emergency stop sent to all motors");
}

void UnifiedMotorGUI::onMotorSelected(int index)
{
  if (index >= 0 && index < motor_tabs_->count()) {
    motor_tabs_->setCurrentIndex(index);
  }
}

void UnifiedMotorGUI::onMotorStatusChanged(const QString & name, bool connected)
{
  logMessage(name + (connected ? " connected" : " disconnected"));
}

void UnifiedMotorGUI::addMotor(const MotorConfig & config)
{
  // Check if name already exists
  if (motor_widgets_.find(config.name) != motor_widgets_.end()) {
    QMessageBox::warning(this, "Error", "Motor with this name already exists");
    return;
  }
  
  motor_configs_.push_back(config);
  
  auto * widget = new SingleMotorWidget(config, this);
  motor_widgets_[config.name] = widget;
  
  connect(widget, &SingleMotorWidget::connectionChanged,
          [this, name = config.name](bool connected) {
            onMotorStatusChanged(name, connected);
          });
  connect(widget, &SingleMotorWidget::statusMessage,
          this, &UnifiedMotorGUI::logMessage);
  connect(widget, &SingleMotorWidget::errorMessage,
          [this](const QString & error) {
            status_label_->setText("Error: " + error);
          });
  
  motor_tabs_->addTab(widget, config.name);
  motor_list_->addItem(config.name);
  
  logMessage("Added motor: " + config.name);
}

void UnifiedMotorGUI::removeMotor(const QString & name)
{
  auto it = motor_widgets_.find(name);
  if (it == motor_widgets_.end()) return;
  
  // Disconnect first
  if (it->second) {
    it->second->disconnectMotor();
  }
  
  // Find and remove tab
  for (int i = 0; i < motor_tabs_->count(); ++i) {
    if (motor_tabs_->tabText(i) == name) {
      motor_tabs_->removeTab(i);
      break;
    }
  }
  
  // Find and remove from list
  for (int i = 0; i < motor_list_->count(); ++i) {
    if (motor_list_->item(i)->text() == name) {
      delete motor_list_->takeItem(i);
      break;
    }
  }
  
  // Remove from configs
  motor_configs_.erase(
    std::remove_if(motor_configs_.begin(), motor_configs_.end(),
                   [&name](const MotorConfig & c) { return c.name == name; }),
    motor_configs_.end());
  
  delete it->second;
  motor_widgets_.erase(it);
  
  logMessage("Removed motor: " + name);
}

void UnifiedMotorGUI::updateMotorList()
{
  motor_list_->clear();
  for (const auto & config : motor_configs_) {
    motor_list_->addItem(config.name);
  }
}

void UnifiedMotorGUI::logMessage(const QString & message)
{
  status_label_->setText(message);
}

// ============== AddMotorDialog Implementation ==============

AddMotorDialog::AddMotorDialog(QWidget * parent)
: QDialog(parent)
{
  setWindowTitle("Add Motor");
  setMinimumWidth(400);
  setupUI();
}

void AddMotorDialog::setupUI()
{
  auto * layout = new QFormLayout(this);
  
  name_edit_ = new QLineEdit(this);
  name_edit_->setPlaceholderText("Enter motor name");
  layout->addRow("Name:", name_edit_);
  
  type_combo_ = new QComboBox(this);
  type_combo_->addItem("ZD Motor (变频电机)", static_cast<int>(MotorConfig::MotorType::ZD_MOTOR));
  type_combo_->addItem("LC Servo (伺服电机)", static_cast<int>(MotorConfig::MotorType::LC_SERVO));
  type_combo_->addItem("LC Stepper (步进电机)", static_cast<int>(MotorConfig::MotorType::LC_STEPPER));
  connect(type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &AddMotorDialog::onMotorTypeChanged);
  layout->addRow("Type:", type_combo_);
  
  auto * port_layout = new QHBoxLayout();
  port_edit_ = new QLineEdit("/dev/ttyUSB0", this);
  browse_port_btn_ = new QPushButton("...", this);
  browse_port_btn_->setMaximumWidth(30);
  connect(browse_port_btn_, &QPushButton::clicked, this, &AddMotorDialog::onBrowsePortClicked);
  port_layout->addWidget(port_edit_);
  port_layout->addWidget(browse_port_btn_);
  layout->addRow("Port:", port_layout);
  
  baud_combo_ = new QComboBox(this);
  baud_combo_->addItem("2400", 2400);
  baud_combo_->addItem("4800", 4800);
  baud_combo_->addItem("9600", 9600);
  baud_combo_->addItem("19200", 19200);
  baud_combo_->addItem("38400", 38400);
  baud_combo_->addItem("57600", 57600);
  baud_combo_->addItem("115200", 115200);
  baud_combo_->addItem("256000", 256000);
  baud_combo_->setCurrentText("19200");
  layout->addRow("Baud Rate:", baud_combo_);
  
  parity_combo_ = new QComboBox(this);
  parity_combo_->addItem("None", 0);
  parity_combo_->addItem("Even", 1);
  parity_combo_->addItem("Odd", 2);
  layout->addRow("Parity:", parity_combo_);
  
  address_spin_ = new QSpinBox(this);
  address_spin_->setRange(1, 247);
  address_spin_->setValue(1);
  layout->addRow("Slave Address:", address_spin_);
  
  auto * btn_box = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  connect(btn_box, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(btn_box, &QDialogButtonBox::rejected, this, &QDialog::reject);
  layout->addRow(btn_box);
  
  updateDefaultSettings();
}

void AddMotorDialog::onMotorTypeChanged(int /*index*/)
{
  updateDefaultSettings();
}

void AddMotorDialog::updateDefaultSettings()
{
  auto type = static_cast<MotorConfig::MotorType>(type_combo_->currentData().toInt());
  
  switch (type) {
    case MotorConfig::MotorType::ZD_MOTOR:
      baud_combo_->setCurrentText("19200");
      parity_combo_->setCurrentIndex(0);  // None
      break;
    case MotorConfig::MotorType::LC_SERVO:
      baud_combo_->setCurrentText("19200");
      parity_combo_->setCurrentIndex(1);  // Even
      break;
    case MotorConfig::MotorType::LC_STEPPER:
      baud_combo_->setCurrentText("115200");
      parity_combo_->setCurrentIndex(0);  // None
      break;
  }
}

void AddMotorDialog::onBrowsePortClicked()
{
  // List available serial ports
  QDir dev_dir("/dev");
  QStringList filters;
  filters << "ttyUSB*" << "ttyACM*" << "ttyS*";
  QStringList ports = dev_dir.entryList(filters, QDir::System);
  
  if (ports.isEmpty()) {
    QMessageBox::information(this, "Serial Ports", "No serial ports found");
    return;
  }
  
  // Show selection dialog
  QStringList full_paths;
  for (const QString & port : ports) {
    full_paths << "/dev/" + port;
  }
  
  bool ok;
  QString selected = QInputDialog::getItem(this, "Select Port", 
    "Available ports:", full_paths, 0, false, &ok);
  if (ok && !selected.isEmpty()) {
    port_edit_->setText(selected);
  }
}

MotorConfig AddMotorDialog::getConfig() const
{
  MotorConfig config;
  config.name = name_edit_->text();
  config.type = static_cast<MotorConfig::MotorType>(type_combo_->currentData().toInt());
  config.port = port_edit_->text();
  config.baud_rate = baud_combo_->currentData().toInt();
  config.parity = parity_combo_->currentData().toInt();
  config.slave_address = static_cast<uint8_t>(address_spin_->value());
  return config;
}

