#include "rs485_interface/lc_servo_motor/lc_servo_motor_multi.hpp"
#include <algorithm>

namespace rs485_interface
{

LcServoMotorMulti::LcServoMotorMulti(
  const std::string & device_path,
  RS485ClientServo::BaudRate baud_rate,
  RS485ClientServo::Parity parity,
  int timeout_ms)
: device_path_(device_path),
  baud_rate_(baud_rate),
  parity_(parity),
  timeout_ms_(timeout_ms)
{
  rs485_client_ = std::make_shared<RS485ClientServo>(device_path_, baud_rate_, parity_, timeout_ms_);
}

LcServoMotorMulti::~LcServoMotorMulti()
{
  close();
}

bool LcServoMotorMulti::addMotor(uint8_t address, const std::string & name)
{
  if (address < 1 || address > 247) {
    last_error_ = "Invalid slave address (must be 1-247)";
    return false;
  }

  if (motors_.find(address) != motors_.end()) {
    last_error_ = "Motor with address " + std::to_string(address) + " already exists";
    return false;
  }

  if (!rs485_client_) {
    rs485_client_ = std::make_shared<RS485ClientServo>(device_path_, baud_rate_, parity_, timeout_ms_);
  }

  auto motor = std::make_shared<LcServoMotor>(rs485_client_, address);
  motors_[address] = motor;
  
  if (!name.empty()) {
    motor_names_[address] = name;
  } else {
    motor_names_[address] = "Motor " + std::to_string(address);
  }

  // If connection is already open, initialize the motor
  if (isOpen()) {
    if (!motor->initialize()) {
      // Log warning but don't fail - user can initialize manually later
      // Keep the warning in last_error_ so user can see it
      last_error_ = "Warning: Failed to initialize motor " + std::to_string(address) + 
                    ": " + motor->getLastError();
      // Still return true since motor was added successfully
      return true;
    } else {
      // Initialization successful, clear any previous errors
      last_error_.clear();
    }
  } else {
    // Connection not open yet, clear error
    last_error_.clear();
  }

  return true;
}

bool LcServoMotorMulti::removeMotor(uint8_t address)
{
  auto it = motors_.find(address);
  if (it == motors_.end()) {
    last_error_ = "Motor with address " + std::to_string(address) + " not found";
    return false;
  }

  // Stop and disable motor before removing
  if (isOpen()) {
    it->second->setDirection(LcServoMotor::Direction::STOP);
    it->second->setEnable(LcServoMotor::EnableState::DISABLE);
  }

  motors_.erase(it);
  motor_names_.erase(address);
  last_error_.clear();
  return true;
}

std::shared_ptr<LcServoMotor> LcServoMotorMulti::getMotor(uint8_t address)
{
  auto it = motors_.find(address);
  if (it == motors_.end()) {
    return nullptr;
  }
  return it->second;
}

std::vector<uint8_t> LcServoMotorMulti::getMotorAddresses() const
{
  std::vector<uint8_t> addresses;
  addresses.reserve(motors_.size());
  for (const auto & pair : motors_) {
    addresses.push_back(pair.first);
  }
  std::sort(addresses.begin(), addresses.end());
  return addresses;
}

std::string LcServoMotorMulti::getMotorName(uint8_t address) const
{
  auto it = motor_names_.find(address);
  if (it == motor_names_.end()) {
    return "";
  }
  return it->second;
}

bool LcServoMotorMulti::setMotorName(uint8_t address, const std::string & name)
{
  if (motors_.find(address) == motors_.end()) {
    last_error_ = "Motor with address " + std::to_string(address) + " not found";
    return false;
  }
  motor_names_[address] = name;
  last_error_.clear();
  return true;
}

bool LcServoMotorMulti::open()
{
  if (!rs485_client_) {
    last_error_ = "RS485 client not initialized";
    return false;
  }

  if (!rs485_client_->open()) {
    last_error_ = "Failed to open RS485 port: " + rs485_client_->getLastError();
    return false;
  }

  // Initialize all existing motors if connection is opened
  for (auto & pair : motors_) {
    if (!pair.second->initialize()) {
      // Log error but continue
      last_error_ = "Warning: Failed to initialize motor " + std::to_string(pair.first) + 
                    ": " + pair.second->getLastError();
    }
  }

  last_error_.clear();
  return true;
}

void LcServoMotorMulti::close()
{
  if (rs485_client_ && rs485_client_->isOpen()) {
    // Stop and disable all motors before closing
    stopAllMotors();
    disableAllMotors();
    rs485_client_->close();
  }
}

bool LcServoMotorMulti::isOpen() const
{
  if (!rs485_client_) {
    return false;
  }
  return rs485_client_->isOpen();
}

std::map<uint8_t, bool> LcServoMotorMulti::initializeAllMotors()
{
  std::map<uint8_t, bool> results;
  for (auto & pair : motors_) {
    results[pair.first] = pair.second->initializeSpeedControl();
  }
  return results;
}

bool LcServoMotorMulti::initializeMotor(uint8_t address)
{
  auto motor = getMotor(address);
  if (!motor) {
    last_error_ = "Motor with address " + std::to_string(address) + " not found";
    return false;
  }
  return motor->initializeSpeedControl();
}

std::map<uint8_t, bool> LcServoMotorMulti::enableAllMotors()
{
  std::map<uint8_t, bool> results;
  for (auto & pair : motors_) {
    results[pair.first] = pair.second->setEnable(LcServoMotor::EnableState::ENABLE);
  }
  return results;
}

std::map<uint8_t, bool> LcServoMotorMulti::disableAllMotors()
{
  std::map<uint8_t, bool> results;
  for (auto & pair : motors_) {
    results[pair.first] = pair.second->setEnable(LcServoMotor::EnableState::DISABLE);
  }
  return results;
}

std::map<uint8_t, bool> LcServoMotorMulti::setSpeedAllMotors(double speed_rpm)
{
  std::map<uint8_t, bool> results;
  for (auto & pair : motors_) {
    results[pair.first] = pair.second->setSpeedRPM(speed_rpm);
  }
  return results;
}

std::map<uint8_t, bool> LcServoMotorMulti::setDirectionAllMotors(LcServoMotor::Direction direction)
{
  std::map<uint8_t, bool> results;
  for (auto & pair : motors_) {
    results[pair.first] = pair.second->setDirection(direction);
  }
  return results;
}

std::map<uint8_t, bool> LcServoMotorMulti::stopAllMotors()
{
  return setDirectionAllMotors(LcServoMotor::Direction::STOP);
}

std::string LcServoMotorMulti::getLastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (rs485_client_) {
    return rs485_client_->getLastError();
  }
  return "No error";
}

}  // namespace rs485_interface

