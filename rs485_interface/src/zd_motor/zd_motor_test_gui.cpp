#include "rs485_interface/zd_motor/zd_motor_test_gui.hpp"
#include "rs485_interface/lc_servo_motor/rs485_client.hpp"
#include "rs485_interface/zd_motor/zd_motor.hpp"

#include <QMessageBox>
#include <iostream>
#include <thread>
#include <chrono>

ZdMotorTestGUI::ZdMotorTestGUI(QWidget * parent)
: QWidget(parent),
  is_connected_(false)
{
  setupUI();
  updateConnectionStatus(false);
  
  // Setup timer for reading status (update every 500ms)
  status_update_timer_ = new QTimer(this);
  connect(status_update_timer_, &QTimer::timeout, this, &ZdMotorTestGUI::onUpdateStatus);
}

ZdMotorTestGUI::~ZdMotorTestGUI()
{
  if (is_connected_) {
    onDisconnectClicked();
  }
}

void ZdMotorTestGUI::setupUI()
{
  setWindowTitle("ZD Motor Control Test");
  setMinimumSize(600, 600);

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

  // ID Management group
  id_management_group_ = new QGroupBox("ID Management", this);
  QVBoxLayout * id_layout = new QVBoxLayout();

  QHBoxLayout * scan_id_layout = new QHBoxLayout();
  scan_id_btn_ = new QPushButton("Scan ID (0-10)", this);
  scan_id_layout->addWidget(scan_id_btn_);
  id_layout->addLayout(scan_id_layout);

  QHBoxLayout * read_id_layout = new QHBoxLayout();
  read_id_btn_ = new QPushButton("Read Current ID", this);
  current_id_label_ = new QLabel("--", this);
  current_id_label_->setStyleSheet("font-weight: bold;");
  read_id_layout->addWidget(read_id_btn_);
  read_id_layout->addWidget(new QLabel("Current ID:", this));
  read_id_layout->addWidget(current_id_label_);
  id_layout->addLayout(read_id_layout);

  QHBoxLayout * set_id_layout = new QHBoxLayout();
  set_id_layout->addWidget(new QLabel("Set ID:", this));
  set_id_spin_ = new QSpinBox(this);
  set_id_spin_->setRange(1, 247);
  set_id_spin_->setValue(1);
  set_id_layout->addWidget(set_id_spin_);
  set_id_btn_ = new QPushButton("Set ID", this);
  set_id_layout->addWidget(set_id_btn_);
  id_layout->addLayout(set_id_layout);

  id_management_group_->setLayout(id_layout);
  main_layout_->addWidget(id_management_group_);

  // Control group
  control_group_ = new QGroupBox("Motor Control", this);
  QVBoxLayout * control_layout = new QVBoxLayout();

  // Direction control buttons
  QHBoxLayout * direction_layout = new QHBoxLayout();
  forward_btn_ = new QPushButton("Forward", this);
  reverse_btn_ = new QPushButton("Reverse", this);
  stop_btn_ = new QPushButton("Stop", this);
  fault_reset_btn_ = new QPushButton("Fault Reset", this);
  direction_layout->addWidget(forward_btn_);
  direction_layout->addWidget(reverse_btn_);
  direction_layout->addWidget(stop_btn_);
  direction_layout->addWidget(fault_reset_btn_);
  control_layout->addLayout(direction_layout);

  // Speed control
  QHBoxLayout * speed_layout = new QHBoxLayout();
  speed_layout->addWidget(new QLabel("Speed (RPM):", this));
  speed_spin_ = new QSpinBox(this);
  speed_spin_->setRange(0, 10000);
  speed_spin_->setValue(3000);
  speed_label_ = new QLabel("3000", this);
  speed_layout->addWidget(speed_spin_);
  speed_layout->addWidget(speed_label_);
  control_layout->addLayout(speed_layout);

  control_group_->setLayout(control_layout);
  main_layout_->addWidget(control_group_);

  // Status group
  status_group_ = new QGroupBox("Status", this);
  QVBoxLayout * status_layout = new QVBoxLayout();

  read_status_btn_ = new QPushButton("Read Status", this);
  status_layout->addWidget(read_status_btn_);

  QHBoxLayout * status_word1_layout = new QHBoxLayout();
  status_word1_layout->addWidget(new QLabel("Status Word 1:", this));
  status_word1_label_ = new QLabel("--", this);
  status_word1_label_->setStyleSheet("font-weight: bold;");
  status_word1_layout->addWidget(status_word1_label_);
  status_layout->addLayout(status_word1_layout);

  QHBoxLayout * status_word2_layout = new QHBoxLayout();
  status_word2_layout->addWidget(new QLabel("Status Word 2:", this));
  status_word2_label_ = new QLabel("--", this);
  status_word2_label_->setStyleSheet("font-weight: bold;");
  status_word2_layout->addWidget(status_word2_label_);
  status_layout->addLayout(status_word2_layout);

  status_label_ = new QLabel("Disconnected", this);
  error_label_ = new QLabel("", this);
  error_label_->setStyleSheet("color: red;");
  status_layout->addWidget(status_label_);
  status_layout->addWidget(error_label_);

  status_group_->setLayout(status_layout);
  main_layout_->addWidget(status_group_);

  main_layout_->addStretch();

  // Connect signals
  connect(connect_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onDisconnectClicked);
  connect(scan_id_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onScanIdClicked);
  connect(read_id_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onReadIdClicked);
  connect(set_id_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onSetIdClicked);
  connect(forward_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onForwardClicked);
  connect(reverse_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onReverseClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onStopClicked);
  connect(fault_reset_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onFaultResetClicked);
  connect(speed_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          this, &ZdMotorTestGUI::onSpeedChanged);
  connect(read_status_btn_, &QPushButton::clicked, this, &ZdMotorTestGUI::onReadStatusClicked);
}

void ZdMotorTestGUI::onConnectClicked()
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

  // Create RS485 client with NONE parity (N-8-1 for ZD Motor)
  rs485_client_ = std::make_shared<rs485_interface::RS485ClientServo>(
    port, baud_rate, rs485_interface::RS485ClientServo::Parity::NONE, 2000);
  
  if (!rs485_client_->open()) {
    std::string error = "Failed to open port: " + rs485_client_->getLastError();
    QMessageBox::critical(this, "Connection Error", QString::fromStdString(error));
    logMessage(error);
    return;
  }

  // Create motor
  motor_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, address);
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
  status_update_timer_->start(500);  // Update status every 500ms
  logMessage("Connected successfully");
}

void ZdMotorTestGUI::onDisconnectClicked()
{
  status_update_timer_->stop();
  
  if (motor_) {
    motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP);
    motor_.reset();
  }
  if (rs485_client_) {
    rs485_client_->close();
    rs485_client_.reset();
  }
  is_connected_ = false;
  updateConnectionStatus(false);
  status_word1_label_->setText("--");
  status_word2_label_->setText("--");
  current_id_label_->setText("--");
  logMessage("Disconnected");
}

void ZdMotorTestGUI::onScanIdClicked()
{
  if (!rs485_client_ || !is_connected_) {
    QMessageBox::warning(this, "Error", "Please connect first");
    return;
  }

  logMessage("Scanning for motor ID (0-10)...");
  scan_id_btn_->setEnabled(false);
  
  uint8_t found_id = 0;
  if (rs485_interface::ZdMotor::scanMotorId(rs485_client_, found_id, 0, 10)) {
    current_id_label_->setText(QString::number(found_id));
    address_spin_->setValue(found_id);
    logMessage("Found motor at ID: " + std::to_string(found_id));
    
    // Recreate motor with found ID
    motor_ = std::make_shared<rs485_interface::ZdMotor>(rs485_client_, found_id);
    if (motor_->initialize()) {
      logMessage("Motor reinitialized with ID: " + std::to_string(found_id));
    }
  } else {
    QMessageBox::warning(this, "Scan Failed", "Could not find motor in ID range 0-10");
    logMessage("ID scan failed");
  }
  
  scan_id_btn_->setEnabled(true);
}

void ZdMotorTestGUI::onReadIdClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  
  uint8_t id = 0;
  if (motor_->readMotorId(id)) {
    current_id_label_->setText(QString::number(id));
    address_spin_->setValue(id);
    logMessage("Current motor ID: " + std::to_string(id));
    error_label_->setText("");
  } else {
    std::string error = "Failed to read ID: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onSetIdClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  
  uint8_t new_id = static_cast<uint8_t>(set_id_spin_->value());
  
  QMessageBox::StandardButton reply = QMessageBox::question(
    this, "Set Motor ID", 
    QString("Are you sure you want to set motor ID to %1?\n\nNote: Motor must be in non-running state.").arg(new_id),
    QMessageBox::Yes | QMessageBox::No);
  
  if (reply == QMessageBox::Yes) {
    if (motor_->setMotorId(new_id)) {
      current_id_label_->setText(QString::number(new_id));
      address_spin_->setValue(new_id);
      logMessage("Motor ID set to: " + std::to_string(new_id));
      error_label_->setText("");
    } else {
      std::string error = "Failed to set ID: " + motor_->getLastError();
      error_label_->setText(QString::fromStdString(error));
      logMessage(error);
    }
  }
}

void ZdMotorTestGUI::onForwardClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::FORWARD)) {
    logMessage("Set to Forward");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set forward: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onReverseClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::REVERSE)) {
    logMessage("Set to Reverse");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set reverse: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onStopClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::STOP)) {
    logMessage("Motor stopped");
    error_label_->setText("");
  } else {
    std::string error = "Failed to stop motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onFaultResetClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setControlCommand(rs485_interface::ZdMotor::ControlCommand::FAULT_RESET)) {
    logMessage("Fault reset");
    error_label_->setText("");
  } else {
    std::string error = "Failed to reset fault: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onSpeedChanged(int value)
{
  speed_label_->setText(QString::number(value));
  if (!motor_ || !is_connected_) {
    return;
  }
  
  if (motor_->setSpeedRPM(static_cast<uint16_t>(value))) {
    error_label_->setText("");
    logMessage("Speed set to: " + std::to_string(value) + " RPM");
  } else {
    std::string error = "Failed to set speed: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void ZdMotorTestGUI::onReadStatusClicked()
{
  onUpdateStatus();
}

void ZdMotorTestGUI::onUpdateStatus()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  
  uint16_t status1 = 0, status2 = 0;
  bool success1 = motor_->readStatusWord1(status1);
  bool success2 = motor_->readStatusWord2(status2);
  
  if (success1) {
    status_word1_label_->setText("0x" + QString::number(status1, 16).toUpper());
  } else {
    status_word1_label_->setText("Error");
  }
  
  if (success2) {
    status_word2_label_->setText("0x" + QString::number(status2, 16).toUpper());
  } else {
    status_word2_label_->setText("Error");
  }
}

void ZdMotorTestGUI::updateConnectionStatus(bool connected)
{
  if (connected) {
    status_label_->setText("Connected");
    status_label_->setStyleSheet("color: green;");
    connect_btn_->setEnabled(false);
    disconnect_btn_->setEnabled(true);
    port_edit_->setEnabled(false);
    baud_combo_->setEnabled(false);
    address_spin_->setEnabled(false);
    scan_id_btn_->setEnabled(true);
    read_id_btn_->setEnabled(true);
    set_id_btn_->setEnabled(true);
    forward_btn_->setEnabled(true);
    reverse_btn_->setEnabled(true);
    stop_btn_->setEnabled(true);
    fault_reset_btn_->setEnabled(true);
    speed_spin_->setEnabled(true);
    read_status_btn_->setEnabled(true);
  } else {
    status_label_->setText("Disconnected");
    status_label_->setStyleSheet("color: red;");
    connect_btn_->setEnabled(true);
    disconnect_btn_->setEnabled(false);
    port_edit_->setEnabled(true);
    baud_combo_->setEnabled(true);
    address_spin_->setEnabled(true);
    scan_id_btn_->setEnabled(false);
    read_id_btn_->setEnabled(false);
    set_id_btn_->setEnabled(false);
    forward_btn_->setEnabled(false);
    reverse_btn_->setEnabled(false);
    stop_btn_->setEnabled(false);
    fault_reset_btn_->setEnabled(false);
    speed_spin_->setEnabled(false);
    read_status_btn_->setEnabled(false);
  }
  error_label_->setText("");
}

void ZdMotorTestGUI::logMessage(const std::string & message)
{
  std::cout << "[ZD Motor Test] " << message << std::endl;
}

