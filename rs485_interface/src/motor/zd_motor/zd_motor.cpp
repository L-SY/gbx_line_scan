#include "rs485_interface/motor/zd_motor/zd_motor.hpp"
#include "rs485_interface/common/rs485_client.hpp"
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>

namespace rs485_interface
{

ZdMotor::ZdMotor(
  std::shared_ptr<RS485ClientServo> rs485_client,
  uint8_t slave_address)
: RS485DeviceBaseServo(rs485_client, slave_address)
{
}

bool ZdMotor::initialize()
{
  if (!rs485_client_) {
    last_error_ = "RS485 client is null";
    return false;
  }

  if (!rs485_client_->isOpen()) {
    last_error_ = "RS485 client is not open";
    return false;
  }

  // Motor is ready
  last_error_.clear();
  return true;
}

std::string ZdMotor::getDeviceType() const
{
  return "ZD Motor (Modbus RTU)";
}

bool ZdMotor::scanMotorId(
  std::shared_ptr<RS485ClientServo> rs485_client,
  uint8_t & found_id,
  uint8_t start_id,
  uint8_t end_id)
{
  if (!rs485_client || !rs485_client->isOpen()) {
    return false;
  }

  // Scan through ID range
  for (uint8_t id = start_id; id <= end_id; ++id) {
    // Try to read F08.00 register (slave address register)
    std::vector<uint16_t> result;
    if (rs485_client->readHoldingRegisters(id, REG_SLAVE_ADDRESS, 1, result)) {
      if (!result.empty()) {
        uint16_t read_id = result[0];
        // Check if the read ID matches the address we used
        // The register should contain the slave address
        if (read_id == id || (read_id >= 1 && read_id <= 247)) {
          found_id = static_cast<uint8_t>(read_id);
          return true;
        }
      }
    }
    // Small delay between scans
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return false;
}

bool ZdMotor::readMotorId(uint8_t & id)
{
  uint16_t value = 0;
  if (!readRegister(REG_SLAVE_ADDRESS, value)) {
    return false;
  }
  id = static_cast<uint8_t>(value);
  return true;
}

bool ZdMotor::setMotorId(uint8_t new_id)
{
  if (new_id > 247) {
    last_error_ = "Invalid motor ID (must be 0-247)";
    return false;
  }

  // First, stop the motor to ensure it's in non-running state
  setControlCommand(ControlCommand::STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Enable write operation for F00-F10 group function codes (200EH = 1)
  // This is required before writing to F08.00 (slave address register)
  if (!writeRegister(REG_WRITE_ENABLE, 1)) {
    last_error_ = "Failed to enable write operation (200EH): " + last_error_;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Write new ID to F08.00 register
  if (!writeRegister(REG_SLAVE_ADDRESS, new_id)) {
    // Try to disable write operation even if write failed
    writeRegister(REG_WRITE_ENABLE, 0);
    return false;
  }

  // Wait for device to process and save
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Disable write operation for security (200EH = 0)
  writeRegister(REG_WRITE_ENABLE, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Update our slave address
  slave_address_ = new_id;

  last_error_.clear();
  return true;
}

bool ZdMotor::readRegister(uint16_t register_address, uint16_t & value)
{
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, register_address, 1, result)) {
    last_error_ = "Failed to read register: " + rs485_client_->getLastError();
    return false;
  }
  if (result.empty()) {
    last_error_ = "No data received";
    return false;
  }
  value = result[0];
  last_error_.clear();
  return true;
}

bool ZdMotor::writeRegister(uint16_t register_address, uint16_t value)
{
  if (!rs485_client_->writeSingleRegister(slave_address_, register_address, value)) {
    last_error_ = "Failed to write register: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool ZdMotor::setControlCommand(ControlCommand command)
{
  uint16_t value = static_cast<uint16_t>(command);
  if (!writeRegister(REG_CONTROL_CMD, value)) {
    return false;
  }
  // Wait for device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  last_error_.clear();
  return true;
}

bool ZdMotor::setSpeedRPM(uint16_t speed_rpm)
{
  if (!writeRegister(REG_SPEED_SETTING, speed_rpm)) {
    return false;
  }
  // Wait for device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  last_error_.clear();
  return true;
}

bool ZdMotor::readStatusWord1(uint16_t & status)
{
  return readRegister(REG_STATUS_WORD1, status);
}

bool ZdMotor::readStatusWord2(uint16_t & status)
{
  return readRegister(REG_STATUS_WORD2, status);
}

}  // namespace rs485_interface

