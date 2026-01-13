#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor.hpp"
#include <cstring>
#include <thread>
#include <chrono>

namespace rs485_interface
{

LcServoMotor::LcServoMotor(
  std::shared_ptr<RS485ClientServo> rs485_client,
  uint8_t slave_address)
: RS485DeviceBaseServo(rs485_client, slave_address)
{
}

bool LcServoMotor::initialize()
{
  if (!rs485_client_) {
    last_error_ = "RS485 client is null";
    return false;
  }

  if (!rs485_client_->isOpen()) {
    last_error_ = "RS485 client is not open";
    return false;
  }

  // Motor is ready, but initialization sequence needs to be called explicitly
  last_error_.clear();
  return true;
}

std::string LcServoMotor::getDeviceType() const
{
  return "LC Servo Motor (DS2-P/R compatible)";
}

bool LcServoMotor::readRegister(uint16_t register_address, uint16_t & value)
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

bool LcServoMotor::setModbusMode()
{
  // P02-00 = 9 (Modbus bus mode)
  // First, try to read the current value to check if already set
  uint16_t current_value = 0;
  if (readRegister(REG_P02_00, current_value)) {
    // Address is valid and readable
    if (current_value == 9) {
      // Already in Modbus mode, no need to write
      last_error_.clear();
      return true;
    }
    // Current value is not 9, try to write
    if (rs485_client_->writeSingleRegister(slave_address_, REG_P02_00, 9)) {
      // Wait for device to process
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      last_error_.clear();
      return true;
    }
  }
  
  // If read or write failed, the address might be wrong or device is in a protected state
  // Try alternative address (0-based addressing: 0x0200 - 1 = 0x01FF)
  if (readRegister(0x01FF, current_value)) {
    if (current_value == 9) {
      last_error_.clear();
      return true;
    }
    if (rs485_client_->writeSingleRegister(slave_address_, 0x01FF, 9)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      last_error_.clear();
      return true;
    }
  }
  
  // If all attempts failed, return error
  last_error_ = "Failed to set Modbus mode: " + rs485_client_->getLastError();
  return false;
}

bool LcServoMotor::forceDIEnable()
{
  // P0D-17 = 1 (Force DI enable)
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_17, 1)) {
    last_error_ = "Failed to force DI enable: " + rs485_client_->getLastError();
    return false;
  }
  // Wait for device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  last_error_.clear();
  return true;
}

bool LcServoMotor::setPVMode()
{
  // P10-03 = 3 (PV mode - speed control)
  // First check if already in PV mode
  uint16_t current_value = 0;
  if (readRegister(REG_P10_03, current_value)) {
    if (current_value == 3) {
      // Already in PV mode
      last_error_.clear();
      return true;
    }
  }
  
  // Not in PV mode, try to set it
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P10_03, 3)) {
    last_error_ = "Failed to set PV mode: " + rs485_client_->getLastError();
    return false;
  }
  // Wait for device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Verify it was set correctly
  if (readRegister(REG_P10_03, current_value)) {
    if (current_value != 3) {
      last_error_ = "PV mode verification failed: got " + std::to_string(current_value) + ", expected 3";
      return false;
    }
  }
  
  last_error_.clear();
  return true;
}

bool LcServoMotor::setSpeed(uint32_t speed_prs)
{
  // IMPORTANT: Verify we're in PV mode before setting speed
  // If not in PV mode, speed control won't work and motor will behave like torque mode
  uint16_t current_mode = 0;
  if (readOperatingMode(current_mode)) {
    if (current_mode != 3) {
      last_error_ = "Not in PV mode (current mode: " + std::to_string(current_mode) + 
                    "). Speed control requires PV mode (3). Please initialize speed control first.";
      return false;
    }
  }
  
  // P10-42 is a 32-bit value stored in 2 registers
  // According to the manual example: 01 10 10 2A 00 02 04 4E 20 00 00
  // For 20000 (0x4E20), the data is: 4E 20 00 00
  // This means: LOW word first (4E 20), then HIGH word (00 00)
  // So the device uses little-endian word order (low word at lower address)
  //
  // Register 0x102A gets the LOW 16-bit word
  // Register 0x102B gets the HIGH 16-bit word
  uint16_t low_word = speed_prs & 0xFFFF;   // Low 16-bit word (written to 0x102A)
  uint16_t high_word = (speed_prs >> 16) & 0xFFFF;  // High 16-bit word (written to 0x102B)

  std::vector<uint16_t> values;
  values.push_back(low_word);   // Low 16-bit word (written to 0x102A first)
  values.push_back(high_word);  // High 16-bit word (written to 0x102B second)

  if (!rs485_client_->writeMultipleRegisters(slave_address_, REG_P10_42, values)) {
    last_error_ = "Failed to set speed: " + rs485_client_->getLastError();
    return false;
  }
  
  // Wait for device to process speed setting
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  last_error_.clear();
  return true;
}

bool LcServoMotor::setSpeedRPM(double speed_rpm)
{
  // Use default encoder resolution
  return setSpeedRPM(speed_rpm, DEFAULT_ENCODER_RESOLUTION);
}

bool LcServoMotor::setSpeedRPM(double speed_rpm, double encoder_resolution)
{
  // Convert RPM to Pr/s (pulses per second)
  // Formula: Pr/s = RPM * (encoder_resolution / 60)
  // 
  // Example with default encoder_resolution = 10000:
  // 120 RPM * (10000 / 60) = 120 * 166.67 = 20000 Pr/s
  double speed_prs = speed_rpm * (encoder_resolution / 60.0);
  uint32_t speed_prs_int = static_cast<uint32_t>(speed_prs + 0.5);  // Round to nearest
  
  return setSpeed(speed_prs_int);
}

bool LcServoMotor::setEnable(EnableState enable)
{
  // P0D-18: 507 = enable, 511 = disable
  uint16_t value = static_cast<uint16_t>(enable);
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_18, value)) {
    last_error_ = "Failed to set enable state: " + rs485_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool LcServoMotor::setDirection(Direction direction)
{
  // Important: In speed control mode, we should set speed first, then direction
  // P0D-08: 16 = forward, 32 = reverse, 256 = stop
  uint16_t value = static_cast<uint16_t>(direction);
  if (!rs485_client_->writeSingleRegister(slave_address_, REG_P0D_08, value)) {
    last_error_ = "Failed to set direction: " + rs485_client_->getLastError();
    return false;
  }
  // Wait for device to process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  last_error_.clear();
  return true;
}

bool LcServoMotor::readOperatingMode(uint16_t & mode)
{
  return readRegister(REG_P10_03, mode);
}

bool LcServoMotor::readCurrentSpeed(double & speed_rpm)
{
  // Read P0B-00 (current speed in RPM)
  // Note: According to manual, P0B-00 returns speed in RPM
  std::vector<uint16_t> result;
  if (!rs485_client_->readHoldingRegisters(slave_address_, REG_P0B_00, 1, result)) {
    last_error_ = "Failed to read current speed: " + rs485_client_->getLastError();
    return false;
  }

  if (result.empty()) {
    last_error_ = "No data received";
    return false;
  }

  // The value from P0B-00 should be in RPM
  // But if it's actually in Pr/s or other units, we need to convert
  // For now, assume it's in RPM as per manual
  speed_rpm = static_cast<double>(result[0]);
  
  // If the value seems too high (e.g., > 10000), it might be in Pr/s
  // Convert: RPM = Pr/s * 60 / encoder_resolution
  if (speed_rpm > 10000) {
    // Likely in Pr/s, convert to RPM
    speed_rpm = speed_rpm * 60.0 / DEFAULT_ENCODER_RESOLUTION;
  }
  
  last_error_.clear();
  return true;
}

bool LcServoMotor::initializeSpeedControl()
{
  // Complete initialization sequence for speed control
  // Note: Some devices may already be in the correct mode, so we try to set
  // but don't fail if the device rejects the write (it might already be set)
  
  // Step 1: Set Modbus mode (P02-00 = 9)
  // This might fail if device is already in Modbus mode or in protected state
  // We'll continue anyway and see if other steps work
  setModbusMode();  // Don't fail if this doesn't work, might already be set
  
  // Step 2: Force DI enable (P0D-17 = 1)
  if (!forceDIEnable()) {
    // This is important, so we fail if it doesn't work
    return false;
  }

  // Step 3: Set PV mode (P10-03 = 3)
  if (!setPVMode()) {
    return false;
  }

  last_error_.clear();
  return true;
}

}  // namespace rs485_interface

