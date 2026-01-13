#include "rs485_interface/motor/lc_stepper_motor/lc_stepper_motor_test_gui.hpp"
#include "rs485_interface/common/rs485_client.hpp"
#include "rs485_interface/motor/lc_stepper_motor/lc_stepper_motor.hpp"

#include <QMessageBox>
#include <iostream>

LcStepperMotorTestGUI::LcStepperMotorTestGUI(QWidget * parent)
: QWidget(parent),
  is_connected_(false)
{
  setupUI();
  updateConnectionStatus(false);
}

LcStepperMotorTestGUI::~LcStepperMotorTestGUI()
{
  if (is_connected_) {
    onDisconnectClicked();
  }
}

void LcStepperMotorTestGUI::setupUI()
{
  setWindowTitle("LC Stepper Motor Test");
  setMinimumSize(500, 400);

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
  baud_combo_->addItem("9600", 9600);
  baud_combo_->addItem("19200", 19200);
  baud_combo_->addItem("38400", 38400);
  baud_combo_->addItem("57600", 57600);
  baud_combo_->addItem("115200", 115200);
  baud_combo_->addItem("256000", 256000);
  baud_combo_->setCurrentIndex(4);  // Default to 115200
  baud_layout->addWidget(baud_combo_);
  connection_layout->addLayout(baud_layout);

  QHBoxLayout * address_layout = new QHBoxLayout();
  address_layout->addWidget(new QLabel("Device Address:", this));
  address_spin_ = new QSpinBox(this);
  address_spin_->setRange(1, 255);
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
  control_group_ = new QGroupBox("Motor Control (CL57)", this);
  QVBoxLayout * control_layout = new QVBoxLayout();

  // State control buttons
  QHBoxLayout * state_btn_layout = new QHBoxLayout();
  start_btn_ = new QPushButton("Start", this);
  stop_btn_ = new QPushButton("Stop", this);
  state_btn_layout->addWidget(start_btn_);
  state_btn_layout->addWidget(stop_btn_);
  control_layout->addLayout(state_btn_layout);

  // Position control
  QHBoxLayout * position_layout = new QHBoxLayout();
  position_layout->addWidget(new QLabel("Position (mm):", this));
  position_spin_ = new QDoubleSpinBox(this);
  position_spin_->setRange(-2147483647.0, 2147483647.0);
  position_spin_->setValue(0.0);
  position_spin_->setDecimals(2);
  position_label_ = new QLabel("0.00", this);
  position_layout->addWidget(position_spin_);
  position_layout->addWidget(position_label_);
  control_layout->addLayout(position_layout);

  // Speed control
  QHBoxLayout * speed_layout = new QHBoxLayout();
  speed_layout->addWidget(new QLabel("Speed (RPM):", this));
  speed_spin_ = new QDoubleSpinBox(this);
  speed_spin_->setRange(0.0, 65535.0);
  speed_spin_->setValue(1000.0);
  speed_spin_->setDecimals(1);
  speed_label_ = new QLabel("1000.0", this);
  speed_layout->addWidget(speed_spin_);
  speed_layout->addWidget(speed_label_);
  control_layout->addLayout(speed_layout);

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
  connect(connect_btn_, &QPushButton::clicked, this, &LcStepperMotorTestGUI::onConnectClicked);
  connect(disconnect_btn_, &QPushButton::clicked, this, &LcStepperMotorTestGUI::onDisconnectClicked);
  connect(start_btn_, &QPushButton::clicked, this, &LcStepperMotorTestGUI::onStartClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &LcStepperMotorTestGUI::onStopClicked);
  connect(position_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LcStepperMotorTestGUI::onPositionChanged);
  connect(speed_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &LcStepperMotorTestGUI::onSpeedChanged);
}

void LcStepperMotorTestGUI::onConnectClicked()
{
  std::string port = port_edit_->text().toStdString();
  if (port.empty()) {
    QMessageBox::warning(this, "Error", "Please enter a port name");
    return;
  }

  int baud_rate_value = baud_combo_->currentData().toInt();
  rs485_interface::RS485Client::BaudRate baud_rate;
  switch (baud_rate_value) {
    case 9600:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_9600;
      break;
    case 19200:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_19200;
      break;
    case 38400:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_38400;
      break;
    case 57600:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_57600;
      break;
    case 115200:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_115200;
      break;
    case 256000:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_256000;
      break;
    default:
      baud_rate = rs485_interface::RS485Client::BaudRate::BAUD_115200;
  }

  uint8_t address = static_cast<uint8_t>(address_spin_->value());

  // Create RS485 client (stepper uses NONE parity)
  rs485_client_ = std::make_shared<rs485_interface::RS485Client>(
    port, baud_rate, rs485_interface::RS485Client::Parity::NONE, 1000);
  if (!rs485_client_->open()) {
    std::string error = "Failed to open port: " + rs485_client_->getLastError();
    QMessageBox::critical(this, "Connection Error", QString::fromStdString(error));
    logMessage(error);
    return;
  }

  // Create motor
  motor_ = std::make_shared<rs485_interface::LcStepperMotor>(rs485_client_, address);
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
  logMessage("Connected successfully");
}

void LcStepperMotorTestGUI::onDisconnectClicked()
{
  if (motor_) {
    motor_->setState(rs485_interface::LcStepperMotor::MotorState::STOP);
    motor_.reset();
  }
  if (rs485_client_) {
    rs485_client_->close();
    rs485_client_.reset();
  }
  is_connected_ = false;
  updateConnectionStatus(false);
  logMessage("Disconnected");
}

void LcStepperMotorTestGUI::onStartClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setState(rs485_interface::LcStepperMotor::MotorState::START)) {
    logMessage("Set motor to Start mode");
    error_label_->setText("");
  } else {
    std::string error = "Failed to start motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcStepperMotorTestGUI::onStopClicked()
{
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setState(rs485_interface::LcStepperMotor::MotorState::STOP)) {
    logMessage("Set motor to Stop");
    error_label_->setText("");
  } else {
    std::string error = "Failed to stop motor: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcStepperMotorTestGUI::onPositionChanged(double value)
{
  position_label_->setText(QString::number(value, 'f', 2));
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setPosition(value)) {
    logMessage("Position set to " + std::to_string(value) + " mm");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set position: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcStepperMotorTestGUI::onSpeedChanged(double value)
{
  speed_label_->setText(QString::number(value, 'f', 1));
  if (!motor_ || !is_connected_) {
    return;
  }
  if (motor_->setSpeed(value)) {
    logMessage("Speed set to " + std::to_string(value) + " RPM");
    error_label_->setText("");
  } else {
    std::string error = "Failed to set speed: " + motor_->getLastError();
    error_label_->setText(QString::fromStdString(error));
    logMessage(error);
  }
}

void LcStepperMotorTestGUI::updateConnectionStatus(bool connected)
{
  if (connected) {
    status_label_->setText("Connected");
    status_label_->setStyleSheet("color: green;");
    connect_btn_->setEnabled(false);
    disconnect_btn_->setEnabled(true);
    port_edit_->setEnabled(false);
    baud_combo_->setEnabled(false);
    address_spin_->setEnabled(false);
    start_btn_->setEnabled(true);
    stop_btn_->setEnabled(true);
    position_spin_->setEnabled(true);
    speed_spin_->setEnabled(true);
  } else {
    status_label_->setText("Disconnected");
    status_label_->setStyleSheet("color: red;");
    connect_btn_->setEnabled(true);
    disconnect_btn_->setEnabled(false);
    port_edit_->setEnabled(true);
    baud_combo_->setEnabled(true);
    address_spin_->setEnabled(true);
    start_btn_->setEnabled(false);
    stop_btn_->setEnabled(false);
    position_spin_->setEnabled(false);
    speed_spin_->setEnabled(false);
  }
  error_label_->setText("");
}

void LcStepperMotorTestGUI::logMessage(const std::string & message)
{
  std::cout << "[LC Stepper Motor Test] " << message << std::endl;
}

