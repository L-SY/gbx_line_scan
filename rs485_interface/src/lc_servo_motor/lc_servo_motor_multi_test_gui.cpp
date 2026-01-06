#include "rs485_interface/lc_servo_motor/lc_servo_motor_multi_test_gui.hpp"
#include "rs485_interface/lc_servo_motor/lc_servo_motor_multi.hpp"
#include "rs485_interface/lc_servo_motor/lc_servo_motor.hpp"

#include <QMessageBox>
#include <QScrollArea>
#include <iostream>
#include <thread>
#include <chrono>

// SingleMotorControlWidget implementation
SingleMotorControlWidget::SingleMotorControlWidget(
  uint8_t address,
  const std::string & name,
  QWidget * parent)
: QWidget(parent),
  address_(address),
  name_(name),
  is_initialized_(false)
{
  setupUI();
  
  // Setup timer for reading current speed (update every 500ms)
  speed_update_timer_ = new QTimer(this);
  connect(speed_update_timer_, &QTimer::timeout, this, &SingleMotorControlWidget::onUpdateSpeed);
}

void SingleMotorControlWidget::setupUI()
{
  QVBoxLayout * layout = new QVBoxLayout(this);
  
  // Title
  QString title = QString("Motor %1").arg(address_);
  if (!name_.empty()) {
    title += QString(" (%1)").arg(QString::fromStdString(name_));
  }
  QLabel * title_label = new QLabel(title, this);
  title_label->setStyleSheet("font-weight: bold; font-size: 14px;");
  layout->addWidget(title_label);

  // Initialization button
  init_btn_ = new QPushButton("Initialize Speed Control", this);
  layout->addWidget(init_btn_);

  // Enable/Disable buttons
  QHBoxLayout * enable_layout = new QHBoxLayout();
  enable_btn_ = new QPushButton("Enable Motor", this);
  disable_btn_ = new QPushButton("Disable Motor", this);
  enable_layout->addWidget(enable_btn_);
  enable_layout->addWidget(disable_btn_);
  layout->addLayout(enable_layout);

  // Direction control buttons
  QHBoxLayout * direction_layout = new QHBoxLayout();
  forward_btn_ = new QPushButton("Forward", this);
  reverse_btn_ = new QPushButton("Reverse", this);
  stop_btn_ = new QPushButton("Stop", this);
  direction_layout->addWidget(forward_btn_);
  direction_layout->addWidget(reverse_btn_);
  direction_layout->addWidget(stop_btn_);
  layout->addLayout(direction_layout);

  // Speed control
  QHBoxLayout * speed_layout = new QHBoxLayout();
  speed_layout->addWidget(new QLabel("Speed (RPM):", this));
  speed_spin_ = new QDoubleSpinBox(this);
  speed_spin_->setRange(0.0, 10000.0);
  speed_spin_->setValue(120.0);
  speed_spin_->setDecimals(1);
  speed_spin_->setSingleStep(10.0);
  speed_label_ = new QLabel("120.0", this);
  speed_layout->addWidget(speed_spin_);
  speed_layout->addWidget(speed_label_);
  layout->addLayout(speed_layout);

  // Current speed display
  QHBoxLayout * current_speed_layout = new QHBoxLayout();
  current_speed_layout->addWidget(new QLabel("Current Speed (RPM):", this));
  current_speed_label_ = new QLabel("--", this);
  current_speed_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
  current_speed_layout->addWidget(current_speed_label_);
  layout->addLayout(current_speed_layout);

  // Status
  status_label_ = new QLabel("Not initialized", this);
  error_label_ = new QLabel("", this);
  error_label_->setStyleSheet("color: red;");
  layout->addWidget(status_label_);
  layout->addWidget(error_label_);

  layout->addStretch();

  // Connect signals
  connect(init_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onInitializeClicked);
  connect(enable_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onEnableClicked);
  connect(disable_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onDisableClicked);
  connect(forward_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onForwardClicked);
  connect(reverse_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onReverseClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &SingleMotorControlWidget::onStopClicked);
  connect(speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &SingleMotorControlWidget::onSpeedChanged);

  setEnabled(false);
}

void SingleMotorControlWidget::setMotor(std::shared_ptr<rs485_interface::LcServoMotor> motor)
{
  motor_ = motor;
  if (motor_) {
    setEnabled(true);
    speed_update_timer_->start(500);
  } else {
    setEnabled(false);
    speed_update_timer_->stop();
  }
}

void SingleMotorControlWidget::updateCurrentSpeed()
{
  onUpdateSpeed();
}

void SingleMotorControlWidget::setEnabled(bool enabled)
{
  init_btn_->setEnabled(enabled);
  enable_btn_->setEnabled(enabled && is_initialized_);
  disable_btn_->setEnabled(enabled && is_initialized_);
  forward_btn_->setEnabled(enabled && is_initialized_);
  reverse_btn_->setEnabled(enabled && is_initialized_);
  stop_btn_->setEnabled(enabled && is_initialized_);
  speed_spin_->setEnabled(enabled && is_initialized_);
}

void SingleMotorControlWidget::onInitializeClicked()
{
  if (!motor_) {
    return;
  }
  
  if (motor_->initializeSpeedControl()) {
    is_initialized_ = true;
    status_label_->setText("Initialized");
    status_label_->setStyleSheet("color: green;");
    error_label_->setText("");
    setEnabled(true);
  } else {
    std::string error = "Failed to initialize: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    status_label_->setText("Initialization failed");
    status_label_->setStyleSheet("color: red;");
  }
}

void SingleMotorControlWidget::onEnableClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  if (motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE)) {
    error_label_->setText("");
    emit enableChanged(address_, true);
  } else {
    std::string error = "Failed to enable: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onDisableClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  if (motor_->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE)) {
    error_label_->setText("");
    emit enableChanged(address_, false);
  } else {
    std::string error = "Failed to disable: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onForwardClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::FORWARD)) {
    error_label_->setText("");
    emit directionChanged(address_, static_cast<MotorDirection>(
      static_cast<int>(rs485_interface::LcServoMotor::Direction::FORWARD)));
  } else {
    std::string error = "Failed to set forward: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onReverseClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::REVERSE)) {
    error_label_->setText("");
    emit directionChanged(address_, static_cast<MotorDirection>(
      static_cast<int>(rs485_interface::LcServoMotor::Direction::REVERSE)));
  } else {
    std::string error = "Failed to set reverse: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onStopClicked()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP)) {
    error_label_->setText("");
    emit directionChanged(address_, static_cast<MotorDirection>(
      static_cast<int>(rs485_interface::LcServoMotor::Direction::STOP)));
  } else {
    std::string error = "Failed to stop: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onSpeedChanged(double value)
{
  speed_label_->setText(QString::number(value, 'f', 1));
  if (!motor_ || !is_initialized_) {
    return;
  }
  
  // Verify we're in PV mode before setting speed
  uint16_t mode = 0;
  if (motor_->readOperatingMode(mode)) {
    if (mode != 3) {
      std::string error = "Warning: Not in PV mode (current mode: " + std::to_string(mode) + 
                         "). Please initialize speed control first.";
      error_label_->setText(QString::fromStdString(error));
      return;
    }
  }
  
  // Stop the motor first, set speed, then restart
  motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  if (motor_->setSpeedRPM(value)) {
    error_label_->setText("");
    emit speedChanged(address_, value);
  } else {
    std::string error = "Failed to set speed: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
  }
}

void SingleMotorControlWidget::onUpdateSpeed()
{
  if (!motor_ || !is_initialized_) {
    return;
  }
  
  // First check if we're in PV mode
  uint16_t mode = 0;
  if (motor_->readOperatingMode(mode)) {
    if (mode != 3) {
      current_speed_label_->setText("Not in PV mode (mode: " + QString::number(mode) + ")");
      current_speed_label_->setStyleSheet("color: red; font-weight: bold; font-size: 14px;");
      return;
    }
  }
  
  // Read current speed
  double current_speed = 0.0;
  if (motor_->readCurrentSpeed(current_speed)) {
    current_speed_label_->setText(QString::number(current_speed, 'f', 1));
    if (current_speed > 1000) {
      current_speed_label_->setStyleSheet("color: orange; font-weight: bold; font-size: 14px;");
    } else {
      current_speed_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
    }
  } else {
    current_speed_label_->setText("Error");
    current_speed_label_->setStyleSheet("color: red; font-weight: bold; font-size: 14px;");
  }
}

// LcServoMotorMultiTestGUI implementation
LcServoMotorMultiTestGUI::LcServoMotorMultiTestGUI(QWidget * parent)
: QWidget(parent),
  is_connected_(false)
{
  setupUI();
  updateConnectionStatus(false);
  
  // Setup timer for reading current speeds (update every 500ms)
  speed_update_timer_ = new QTimer(this);
  connect(speed_update_timer_, &QTimer::timeout, this, &LcServoMotorMultiTestGUI::onUpdateAllSpeeds);
}

LcServoMotorMultiTestGUI::~LcServoMotorMultiTestGUI()
{
  if (is_connected_) {
    onDisconnectClicked();
  }
}

void LcServoMotorMultiTestGUI::setupUI()
{
  setWindowTitle("LC Servo Motor Multi-Motor Speed Control Test");
  setMinimumSize(1000, 700);

  main_layout_ = new QVBoxLayout(this);

  // Connection group
  connection_group_ = new QGroupBox("Connection", this);
  QVBoxLayout * connection_layout = new QVBoxLayout();

  QHBoxLayout * port_layout = new QHBoxLayout();
  port_layout->addWidget(new QLabel("Port:", this));
  port_edit_ = new QLineEdit("/dev/ttyUSB0", this);
  port_layout->addWidget(port_edit_);
  connection_layout->addLayout(port_layout);

  QHBoxLayout * baud_layout = new QHBoxLayout();
  baud_layout->addWidget(new QLabel("Baud Rate:", this));
  baud_combo_ = new QComboBox(this);
  baud_combo_->addItem("2400", 2400);
  baud_combo_->addItem("4800", 4800);
  baud_combo_->addItem("9600", 9600);
  baud_combo_->addItem("19200", 19200);
  baud_combo_->setCurrentIndex(3);  // Default to 19200
  baud_layout->addWidget(baud_combo_);
  connection_layout->addLayout(baud_layout);

  QHBoxLayout * connect_btn_layout = new QHBoxLayout();
  connect_btn_ = new QPushButton("Connect", this);
  disconnect_btn_ = new QPushButton("Disconnect", this);
  disconnect_btn_->setEnabled(false);
  connect_btn_layout->addWidget(connect_btn_);
  connect_btn_layout->addWidget(disconnect_btn_);
  connection_layout->addLayout(connect_btn_layout);

  connection_group_->setLayout(connection_layout);
  main_layout_->addWidget(connection_group_);

  // Motor management group
  motor_management_group_ = new QGroupBox("Motor Management", this);
  QVBoxLayout * motor_mgmt_layout = new QVBoxLayout();

  QHBoxLayout * add_motor_layout = new QHBoxLayout();
  add_motor_layout->addWidget(new QLabel("Add Motor - Address:", this));
  add_motor_address_spin_ = new QSpinBox(this);
  add_motor_address_spin_->setRange(1, 247);
  add_motor_address_spin_->setValue(1);
  add_motor_layout->addWidget(add_motor_address_spin_);
  add_motor_layout->addWidget(new QLabel("Name (optional):", this));
  add_motor_name_edit_ = new QLineEdit(this);
  add_motor_name_edit_->setPlaceholderText("Motor name");
  add_motor_layout->addWidget(add_motor_name_edit_);
  add_motor_btn_ = new QPushButton("Add Motor", this);
  add_motor_layout->addWidget(add_motor_btn_);
  motor_mgmt_layout->addLayout(add_motor_layout);

  QHBoxLayout * remove_motor_layout = new QHBoxLayout();
  remove_motor_layout->addWidget(new QLabel("Remove Motor:", this));
  remove_motor_combo_ = new QComboBox(this);
  remove_motor_layout->addWidget(remove_motor_combo_);
  remove_motor_btn_ = new QPushButton("Remove Motor", this);
  remove_motor_layout->addWidget(remove_motor_btn_);
  motor_mgmt_layout->addLayout(remove_motor_layout);

  motor_management_group_->setLayout(motor_mgmt_layout);
  main_layout_->addWidget(motor_management_group_);

  // Global control group
  global_control_group_ = new QGroupBox("Global Control (All Motors)", this);
  QVBoxLayout * global_control_layout = new QVBoxLayout();

  QHBoxLayout * global_buttons_layout = new QHBoxLayout();
  init_all_btn_ = new QPushButton("Initialize All", this);
  enable_all_btn_ = new QPushButton("Enable All", this);
  disable_all_btn_ = new QPushButton("Disable All", this);
  global_buttons_layout->addWidget(init_all_btn_);
  global_buttons_layout->addWidget(enable_all_btn_);
  global_buttons_layout->addWidget(disable_all_btn_);
  global_control_layout->addLayout(global_buttons_layout);

  QHBoxLayout * global_direction_layout = new QHBoxLayout();
  forward_all_btn_ = new QPushButton("Forward All", this);
  reverse_all_btn_ = new QPushButton("Reverse All", this);
  stop_all_btn_ = new QPushButton("Stop All", this);
  global_direction_layout->addWidget(forward_all_btn_);
  global_direction_layout->addWidget(reverse_all_btn_);
  global_direction_layout->addWidget(stop_all_btn_);
  global_control_layout->addLayout(global_direction_layout);

  QHBoxLayout * global_speed_layout = new QHBoxLayout();
  global_speed_layout->addWidget(new QLabel("Speed All (RPM):", this));
  speed_all_spin_ = new QDoubleSpinBox(this);
  speed_all_spin_->setRange(0.0, 10000.0);
  speed_all_spin_->setValue(120.0);
  speed_all_spin_->setDecimals(1);
  speed_all_spin_->setSingleStep(10.0);
  speed_all_label_ = new QLabel("120.0", this);
  global_speed_layout->addWidget(speed_all_spin_);
  global_speed_layout->addWidget(speed_all_label_);
  global_control_layout->addLayout(global_speed_layout);

  global_control_group_->setLayout(global_control_layout);
  main_layout_->addWidget(global_control_group_);

  // Motor tabs
  motor_tabs_ = new QTabWidget(this);
  main_layout_->addWidget(motor_tabs_);

  // Status group
  status_group_ = new QGroupBox("Status", this);
  QVBoxLayout * status_layout = new QVBoxLayout();
  status_label_ = new QLabel("Disconnected", this);
  error_label_ = new QLabel("", this);
  error_label_->setStyleSheet("color: red;");
  status_layout->addWidget(status_label_);
  status_layout->addWidget(error_label_);
  status_group_->setLayout(status_layout);
  main_layout_->addWidget(status_group_);

  // Connect signals
  connect(connect_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onDisconnectClicked);
  connect(add_motor_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onAddMotorClicked);
  connect(remove_motor_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onRemoveMotorClicked);
  connect(init_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onInitializeAllClicked);
  connect(enable_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onEnableAllClicked);
  connect(disable_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onDisableAllClicked);
  connect(forward_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onForwardAllClicked);
  connect(reverse_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onReverseAllClicked);
  connect(stop_all_btn_, &QPushButton::clicked, this, &LcServoMotorMultiTestGUI::onStopAllClicked);
  connect(speed_all_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LcServoMotorMultiTestGUI::onSpeedAllChanged);
}

void LcServoMotorMultiTestGUI::onConnectClicked()
{
  std::string port = port_edit_->text().toStdString();
  if (port.empty()) {
    QMessageBox::warning(this, "Error", "Please enter a port name");
    return;
  }

  int baud_rate_value = baud_combo_->currentData().toInt();
  rs485_interface::RS485ClientServo::BaudRate baud_rate;
  switch (baud_rate_value) {
    case 2400:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_2400;
      break;
    case 4800:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_4800;
      break;
    case 9600:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_9600;
      break;
    case 19200:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_19200;
      break;
    case 38400:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_38400;
      break;
    case 57600:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_57600;
      break;
    default:
      baud_rate = rs485_interface::RS485ClientServo::BaudRate::BAUD_19200;
  }

  // Create multi-motor manager
  motor_multi_ = std::make_shared<rs485_interface::LcServoMotorMulti>(
    port, baud_rate, rs485_interface::RS485ClientServo::Parity::EVEN, 2000);

  if (!motor_multi_->open()) {
    std::string error = "Failed to open connection: " + motor_multi_->getLastError();
    QMessageBox::critical(this, "Connection Error", QString::fromStdString(error));
    logMessage(error);
    motor_multi_.reset();
    return;
  }

  is_connected_ = true;
  updateConnectionStatus(true);
  speed_update_timer_->start(500);
  logMessage("Connected successfully");
}

void LcServoMotorMultiTestGUI::onDisconnectClicked()
{
  speed_update_timer_->stop();
  
  if (motor_multi_) {
    motor_multi_->close();
    motor_multi_.reset();
  }

  // Clear all motor widgets
  for (auto & pair : motor_widgets_) {
    motor_tabs_->removeTab(motor_tabs_->indexOf(pair.second));
    delete pair.second;
  }
  motor_widgets_.clear();
  updateMotorList();

  is_connected_ = false;
  updateConnectionStatus(false);
  logMessage("Disconnected");
}

void LcServoMotorMultiTestGUI::onAddMotorClicked()
{
  if (!motor_multi_ || !is_connected_) {
    QMessageBox::warning(this, "Error", "Please connect first");
    return;
  }

  uint8_t address = static_cast<uint8_t>(add_motor_address_spin_->value());
  std::string name = add_motor_name_edit_->text().toStdString();

  if (motor_multi_->addMotor(address, name)) {
    // Create widget for this motor
    SingleMotorControlWidget * widget = createMotorWidget(address, name);
    if (widget) {
      auto motor = motor_multi_->getMotor(address);
      widget->setMotor(motor);
      motor_widgets_[address] = widget;
      updateMotorList();
      logMessage("Added motor at address " + std::to_string(address));
    }
  } else {
    std::string error = "Failed to add motor: " + motor_multi_->getLastError();
    QMessageBox::warning(this, "Error", QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorMultiTestGUI::onRemoveMotorClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }

  int index = remove_motor_combo_->currentIndex();
  if (index < 0) {
    return;
  }

  uint8_t address = static_cast<uint8_t>(remove_motor_combo_->currentData().toInt());
  
  if (motor_multi_->removeMotor(address)) {
    auto it = motor_widgets_.find(address);
    if (it != motor_widgets_.end()) {
      motor_tabs_->removeTab(motor_tabs_->indexOf(it->second));
      delete it->second;
      motor_widgets_.erase(it);
    }
    updateMotorList();
    logMessage("Removed motor at address " + std::to_string(address));
  } else {
    std::string error = "Failed to remove motor: " + motor_multi_->getLastError();
    QMessageBox::warning(this, "Error", QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorMultiTestGUI::onInitializeAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  
  logMessage("Initializing all motors...");
  auto results = motor_multi_->initializeAllMotors();
  
  int success_count = 0;
  for (const auto & pair : results) {
    if (pair.second) {
      success_count++;
      auto it = motor_widgets_.find(pair.first);
      if (it != motor_widgets_.end()) {
        it->second->setEnabled(true);
      }
    }
  }
  
  logMessage("Initialized " + std::to_string(success_count) + " of " + 
             std::to_string(results.size()) + " motors");
}

void LcServoMotorMultiTestGUI::onEnableAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  motor_multi_->enableAllMotors();
  logMessage("Enabled all motors");
}

void LcServoMotorMultiTestGUI::onDisableAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  motor_multi_->disableAllMotors();
  logMessage("Disabled all motors");
}

void LcServoMotorMultiTestGUI::onForwardAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  motor_multi_->setDirectionAllMotors(rs485_interface::LcServoMotor::Direction::FORWARD);
  logMessage("Set all motors to Forward");
}

void LcServoMotorMultiTestGUI::onReverseAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  motor_multi_->setDirectionAllMotors(rs485_interface::LcServoMotor::Direction::REVERSE);
  logMessage("Set all motors to Reverse");
}

void LcServoMotorMultiTestGUI::onStopAllClicked()
{
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  motor_multi_->stopAllMotors();
  logMessage("Stopped all motors");
}

void LcServoMotorMultiTestGUI::onSpeedAllChanged(double value)
{
  speed_all_label_->setText(QString::number(value, 'f', 1));
  if (!motor_multi_ || !is_connected_) {
    return;
  }
  
  motor_multi_->setSpeedAllMotors(value);
  logMessage("Set all motors speed to " + std::to_string(value) + " RPM");
}

void LcServoMotorMultiTestGUI::onUpdateAllSpeeds()
{
  for (auto & pair : motor_widgets_) {
    pair.second->updateCurrentSpeed();
  }
}

void LcServoMotorMultiTestGUI::updateConnectionStatus(bool connected)
{
  if (connected) {
    status_label_->setText("Connected");
    status_label_->setStyleSheet("color: green;");
    connect_btn_->setEnabled(false);
    disconnect_btn_->setEnabled(true);
    port_edit_->setEnabled(false);
    baud_combo_->setEnabled(false);
    add_motor_btn_->setEnabled(true);
    remove_motor_btn_->setEnabled(true);
    init_all_btn_->setEnabled(true);
    enable_all_btn_->setEnabled(true);
    disable_all_btn_->setEnabled(true);
    forward_all_btn_->setEnabled(true);
    reverse_all_btn_->setEnabled(true);
    stop_all_btn_->setEnabled(true);
    speed_all_spin_->setEnabled(true);
  } else {
    status_label_->setText("Disconnected");
    status_label_->setStyleSheet("color: red;");
    connect_btn_->setEnabled(true);
    disconnect_btn_->setEnabled(false);
    port_edit_->setEnabled(true);
    baud_combo_->setEnabled(true);
    add_motor_btn_->setEnabled(false);
    remove_motor_btn_->setEnabled(false);
    init_all_btn_->setEnabled(false);
    enable_all_btn_->setEnabled(false);
    disable_all_btn_->setEnabled(false);
    forward_all_btn_->setEnabled(false);
    reverse_all_btn_->setEnabled(false);
    stop_all_btn_->setEnabled(false);
    speed_all_spin_->setEnabled(false);
  }
  error_label_->setText("");
}

void LcServoMotorMultiTestGUI::updateMotorList()
{
  remove_motor_combo_->clear();
  if (motor_multi_) {
    auto addresses = motor_multi_->getMotorAddresses();
    for (uint8_t addr : addresses) {
      std::string name = motor_multi_->getMotorName(addr);
      QString display = QString("Address %1").arg(addr);
      if (!name.empty()) {
        display += QString(" (%1)").arg(QString::fromStdString(name));
      }
      remove_motor_combo_->addItem(display, addr);
    }
  }
  remove_motor_btn_->setEnabled(remove_motor_combo_->count() > 0);
}

SingleMotorControlWidget * LcServoMotorMultiTestGUI::createMotorWidget(uint8_t address, const std::string & name)
{
  SingleMotorControlWidget * widget = new SingleMotorControlWidget(address, name, this);
  
  QString tab_name = QString("Motor %1").arg(address);
  if (!name.empty()) {
    tab_name += QString(" (%1)").arg(QString::fromStdString(name));
  }
  
  motor_tabs_->addTab(widget, tab_name);
  
  return widget;
}

void LcServoMotorMultiTestGUI::logMessage(const std::string & message)
{
  std::cout << "[LC Servo Motor Multi Test] " << message << std::endl;
}

