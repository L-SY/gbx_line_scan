#include "rs485_interface/lc_servo_motor/lc_servo_motor_test_gui.hpp"
#include "rs485_interface/lc_servo_motor/rs485_client.hpp"
#include "rs485_interface/lc_servo_motor/lc_servo_motor.hpp"

#include <QMessageBox>
#include <iostream>

LcServoMotorTestGUI::LcServoMotorTestGUI(QWidget * parent)
: QWidget(parent),
  is_connected_(false)
{
  setupUI();
  updateConnectionStatus(false);
  
  // Setup timer for reading current speed (update every 500ms)
  speed_update_timer_ = new QTimer(this);
  connect(speed_update_timer_, &QTimer::timeout, this, &LcServoMotorTestGUI::onUpdateSpeed);
}

LcServoMotorTestGUI::~LcServoMotorTestGUI()
{
  if (is_connected_) {
    onDisconnectClicked();
  }
}

void LcServoMotorTestGUI::setupUI()
{
  setWindowTitle("LC Servo Motor Speed Control Test");
  setMinimumSize(600, 500);

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

  QHBoxLayout * address_layout = new QHBoxLayout();
  address_layout->addWidget(new QLabel("Device Address:", this));
  address_spin_ = new QSpinBox(this);
  address_spin_->setRange(1, 247);
  address_spin_->setValue(1);
  address_layout->addWidget(address_spin_);
  connection_layout->addLayout(address_layout);

  QHBoxLayout * connect_btn_layout = new QHBoxLayout();
  connect_btn_ = new QPushButton("Connect", this);
  disconnect_btn_ = new QPushButton("Disconnect", this);
  disconnect_btn_->setEnabled(false);
  connect_btn_layout->addWidget(connect_btn_);
  connect_btn_layout->addWidget(disconnect_btn_);
  connection_layout->addLayout(connect_btn_layout);

  connection_group_->setLayout(connection_layout);
  main_layout_->addWidget(connection_group_);

  // Control group
  control_group_ = new QGroupBox("Motor Control", this);
  QVBoxLayout * control_layout = new QVBoxLayout();

  // Initialization button
  init_btn_ = new QPushButton("Initialize Speed Control", this);
  control_layout->addWidget(init_btn_);

  // Enable/Disable buttons
  QHBoxLayout * enable_layout = new QHBoxLayout();
  enable_btn_ = new QPushButton("Enable Motor", this);
  disable_btn_ = new QPushButton("Disable Motor", this);
  enable_layout->addWidget(enable_btn_);
  enable_layout->addWidget(disable_btn_);
  control_layout->addLayout(enable_layout);

  // Direction control buttons
  QHBoxLayout * direction_layout = new QHBoxLayout();
  forward_btn_ = new QPushButton("Forward", this);
  reverse_btn_ = new QPushButton("Reverse", this);
  stop_btn_ = new QPushButton("Stop", this);
  direction_layout->addWidget(forward_btn_);
  direction_layout->addWidget(reverse_btn_);
  direction_layout->addWidget(stop_btn_);
  control_layout->addLayout(direction_layout);

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
  control_layout->addLayout(speed_layout);

  // Current speed display
  QHBoxLayout * current_speed_layout = new QHBoxLayout();
  current_speed_layout->addWidget(new QLabel("Current Speed (RPM):", this));
  current_speed_label_ = new QLabel("--", this);
  current_speed_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
  current_speed_layout->addWidget(current_speed_label_);
  control_layout->addLayout(current_speed_layout);

  control_group_->setLayout(control_layout);
  main_layout_->addWidget(control_group_);

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

  main_layout_->addStretch();

  // Connect signals
  connect(connect_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onDisconnectClicked);
  connect(init_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onInitializeClicked);
  connect(enable_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onEnableClicked);
  connect(disable_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onDisableClicked);
  connect(forward_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onForwardClicked);
  connect(reverse_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onReverseClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &LcServoMotorTestGUI::onStopClicked);
  connect(speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LcServoMotorTestGUI::onSpeedChanged);
}

void LcServoMotorTestGUI::onConnectClicked()
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

  uint8_t address = static_cast<uint8_t>(address_spin_->value());

  // Create RS485 client with EVEN parity (default)
  // Use longer timeout for Modbus communication (2000ms)
  rs485_client_ = std::make_shared<rs485_interface::RS485ClientServo>(
    port, baud_rate, rs485_interface::RS485ClientServo::Parity::EVEN, 2000);
  
  if (!rs485_client_->open()) {
    std::string error = "Failed to open port: " + rs485_client_->getLastError();
    QMessageBox::critical(this, "Connection Error", QString::fromStdString(error));
    logMessage(error);
    return;
  }

  // Create motor
  motor_ = std::make_shared<rs485_interface::LcServoMotor>(rs485_client_, address);
  if (!motor_->initialize()) {
    std::string error = "Failed to initialize motor: " + motor_->getLastError();
    QMessageBox::critical(this, "Initialization Error", QString::fromStdString(error));
    logMessage(error);
    rs485_client_->close();
    rs485_client_.reset();
    motor_.reset();
    return;
  }

  is_connected_ = true;
  updateConnectionStatus(true);
  speed_update_timer_->start(500);  // Update speed every 500ms
  logMessage("Connected successfully");
}

void LcServoMotorTestGUI::onDisconnectClicked()
{
  speed_update_timer_->stop();
  
  if (motor_) {
    motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
    motor_->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE);
    motor_.reset();
  }
  if (rs485_client_) {
    rs485_client_->close();
    rs485_client_.reset();
  }
  is_connected_ = false;
  updateConnectionStatus(false);
  current_speed_label_->setText("--");
  logMessage("Disconnected");
}

void LcServoMotorTestGUI::onInitializeClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->initializeSpeedControl()) {
    logMessage("Speed control initialized successfully");
    error_label_->setText("");
  } else {
    std::string error = "Failed to initialize: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onEnableClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE)) {
    logMessage("Motor enabled");
    error_label_->setText("");
  } else {
    std::string error = "Failed to enable motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onDisableClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE)) {
    logMessage("Motor disabled");
    error_label_->setText("");
  } else {
    std::string error = "Failed to disable motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onForwardClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::FORWARD)) {
    logMessage("Set direction to Forward");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set forward: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onReverseClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::REVERSE)) {
    logMessage("Set direction to Reverse");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set reverse: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onStopClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setDirection(rs485_interface::LcServoMotor::Direction::STOP)) {
    logMessage("Motor stopped");
    error_label_->setText("");
  } else {
    std::string error = "Failed to stop motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onSpeedChanged(double value)
{
  speed_label_->setText(QString::number(value, 'f', 1));
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setSpeedRPM(value)) {
    logMessage("Speed set to " + std::to_string(value) + " RPM");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set speed: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcServoMotorTestGUI::onUpdateSpeed()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  double current_speed = 0.0;
  if (motor_->readCurrentSpeed(current_speed)) {
    current_speed_label_->setText(QString::number(current_speed, 'f', 1));
  } else {
    current_speed_label_->setText("Error");
  }
}

void LcServoMotorTestGUI::updateConnectionStatus(bool connected)
{
  if (connected) {
    status_label_->setText("Connected");
    status_label_->setStyleSheet("color: green;");
    connect_btn_->setEnabled(false);
    disconnect_btn_->setEnabled(true);
    port_edit_->setEnabled(false);
    baud_combo_->setEnabled(false);
    address_spin_->setEnabled(false);
    init_btn_->setEnabled(true);
    enable_btn_->setEnabled(true);
    disable_btn_->setEnabled(true);
    forward_btn_->setEnabled(true);
    reverse_btn_->setEnabled(true);
    stop_btn_->setEnabled(true);
    speed_spin_->setEnabled(true);
  } else {
    status_label_->setText("Disconnected");
    status_label_->setStyleSheet("color: red;");
    connect_btn_->setEnabled(true);
    disconnect_btn_->setEnabled(false);
    port_edit_->setEnabled(true);
    baud_combo_->setEnabled(true);
    address_spin_->setEnabled(true);
    init_btn_->setEnabled(false);
    enable_btn_->setEnabled(false);
    disable_btn_->setEnabled(false);
    forward_btn_->setEnabled(false);
    reverse_btn_->setEnabled(false);
    stop_btn_->setEnabled(false);
    speed_spin_->setEnabled(false);
  }
  error_label_->setText("");
}

void LcServoMotorTestGUI::logMessage(const std::string & message)
{
  std::cout << "[LC Servo Motor Test] " << message << std::endl;
}

