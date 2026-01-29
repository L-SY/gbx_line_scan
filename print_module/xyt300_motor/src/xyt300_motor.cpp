#include "xyt300_motor/xyt300_motor.hpp"
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <iomanip>

namespace xyt300_motor
{

Xyt300Motor::Xyt300Motor(
  std::shared_ptr<ModbusClient> modbus_client,
  uint8_t slave_address,
  uint16_t pulses_per_revolution,
  uint16_t gear_ratio,
  uint16_t max_output_rpm)
: modbus_client_(modbus_client),
  slave_address_(slave_address),
  speed_set_(false),
  pulses_per_revolution_(pulses_per_revolution),
  gear_ratio_(gear_ratio),
  max_output_rpm_(max_output_rpm)
{
}

bool Xyt300Motor::initialize(bool /* skip_status_check */)
{
  if (!modbus_client_) {
    last_error_ = "MODBUS client is null";
    return false;
  }

  if (!modbus_client_->isOpen()) {
    last_error_ = "MODBUS client is not open";
    return false;
  }

  // Initialize: Write CRC register to 0xFFFF using broadcast address (0x00)
  // According to protocol, initialization uses broadcast address 0x00,
  // no response is expected or needed
  if (!modbus_client_->writeSingleRegisterNoResponse(0x00, REG_CRC_INIT, 0xFFFF)) {
    last_error_ = "Failed to initialize CRC register: " + modbus_client_->getLastError();
    return false;
  }

  // Wait for device to process initialization
  // Device needs time to initialize after CRC register is set
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  last_error_.clear();
  return true;
}

std::string Xyt300Motor::getDeviceType() const
{
  return "XYT300 Motor";
}

bool Xyt300Motor::readMotorId(uint8_t & id)
{
  // According to protocol, there are two ways to read device address:
  // 1. Normal mode: function code 0x04, register address 0x0010
  // 2. Modbus mode: function code 0x03, register address 0x000D
  // Try Modbus mode first (function code 0x03)
  std::vector<uint16_t> result;
  if (modbus_client_->readHoldingRegisters(slave_address_, REG_SLAVE_ADDRESS_MODBUS, 1, result)) {
    if (!result.empty()) {
      id = static_cast<uint8_t>(result[0] & 0xFF);
      last_error_.clear();
      return true;
    }
  }
  
  // If Modbus mode fails, try normal mode (function code 0x04)
  if (modbus_client_->readInputRegisters(slave_address_, REG_SLAVE_ADDRESS_NORMAL, 1, result)) {
    if (!result.empty()) {
      id = static_cast<uint8_t>(result[0] & 0xFF);
      last_error_.clear();
      return true;
    }
  }
  
  last_error_ = "Failed to read motor ID: " + modbus_client_->getLastError();
  return false;
}

bool Xyt300Motor::setMotorId(uint8_t new_id)
{
  if (new_id == 0 || new_id > 247) {
    last_error_ = "Invalid motor ID (must be 1-247)";
    return false;
  }

  // According to protocol documentation, there are two ways to set device address:
  // Method 1: Function code 0x10 (Write Multiple Registers)
  //   - Register address: 0x00FA
  //   - Data H: 0x00, Data L: device address (xxH)
  //   - Value to write: 0x00XX where XX is the device address
  // Method 2: Function code 0x06 (Write Single Register, Modbus mode)
  //   - Register address: 0x4000
  //   - Data H: 0x00, Data L: device address (xxH)
  //   - Value to write: 0x00XX where XX is the device address
  
  // Try method 1 first (function code 0x10, register 0x00FA)
  // Data format: data H=00H, data L=device address
  std::vector<uint16_t> values;
  values.push_back(static_cast<uint16_t>(new_id));  // Value is 0x00XX where XX is the device address
  
  if (modbus_client_->writeMultipleRegisters(slave_address_, REG_SLAVE_ADDRESS_10H, values)) {
    // Wait for device to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Update internal slave address
    slave_address_ = new_id;
    
    last_error_.clear();
    return true;
  }
  
  // If method 1 fails, try method 2 (function code 0x06, Modbus mode, register 0x4000)
  // Register address 0x4000, data H=00H, data L=device address
  if (modbus_client_->writeSingleRegister(slave_address_, REG_SLAVE_ADDRESS_06H, new_id)) {
    // Wait for device to process
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Update internal slave address
    slave_address_ = new_id;
    
    last_error_.clear();
    return true;
  }

  last_error_ = "Failed to set motor ID (both methods failed): " + modbus_client_->getLastError();
  return false;
}

bool Xyt300Motor::scanMotorIds(std::vector<uint8_t> & found_ids, uint8_t start_address, uint8_t end_address)
{
  found_ids.clear();
  
  if (!modbus_client_ || !modbus_client_->isOpen()) {
    last_error_ = "MODBUS client is not open";
    return false;
  }

  // Save current slave address
  uint8_t original_address = slave_address_;

  // Temporarily reduce timeout for faster scanning
  int original_timeout = modbus_client_->getTimeout();
  modbus_client_->setTimeout(50);  // Reduce to 50ms for faster scanning

  // Scan all possible addresses
  for (uint8_t addr = start_address; addr <= end_address && addr > 0; ++addr) {
    // Try to read device ID from this address
    // Use both methods (Modbus mode and normal mode)
    std::vector<uint16_t> result;
    
    // Try Modbus mode first (function code 0x03, register 0x000D)
    bool found = false;
    if (modbus_client_->readHoldingRegisters(addr, REG_SLAVE_ADDRESS_MODBUS, 1, result)) {
      if (!result.empty()) {
        uint8_t read_id = static_cast<uint8_t>(result[0] & 0xFF);
        // Verify the read ID matches the address we're scanning
        if (read_id == addr) {
          found_ids.push_back(addr);
          found = true;
        }
      }
    }
    
    // If not found, try normal mode (function code 0x04, register 0x0010)
    if (!found) {
      result.clear();
      if (modbus_client_->readInputRegisters(addr, REG_SLAVE_ADDRESS_NORMAL, 1, result)) {
        if (!result.empty()) {
          uint8_t read_id = static_cast<uint8_t>(result[0] & 0xFF);
          if (read_id == addr) {
            found_ids.push_back(addr);
            found = true;
          }
        }
      }
    }
    
    // Minimal delay between scans - only if we found a device
    if (found) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // No delay if not found - speed up scanning
  }

  // Restore original timeout and slave address
  modbus_client_->setTimeout(original_timeout);
  slave_address_ = original_address;

  last_error_.clear();
  return !found_ids.empty();
}

bool Xyt300Motor::readRegister(uint16_t register_address, uint16_t & value)
{
  std::vector<uint16_t> result;
  // Use function code 0x04 (Read Input Registers) for status reading
  // as shown in the protocol documentation
  if (!modbus_client_->readInputRegisters(slave_address_, register_address, 1, result)) {
    last_error_ = "Failed to read register: " + modbus_client_->getLastError();
    return false;
  }
  if (result.empty()) {
    last_error_ = "Empty response";
    return false;
  }
  value = result[0];
  last_error_.clear();
  return true;
}

bool Xyt300Motor::writeRegister(uint16_t register_address, uint16_t value)
{
  if (!modbus_client_->writeSingleRegister(slave_address_, register_address, value)) {
    last_error_ = "Failed to write register: " + modbus_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool Xyt300Motor::writeRegisterNoResponse(uint16_t register_address, uint16_t value)
{
  if (!modbus_client_->writeSingleRegisterNoResponse(slave_address_, register_address, value)) {
    last_error_ = "Failed to write register: " + modbus_client_->getLastError();
    return false;
  }
  last_error_.clear();
  return true;
}

bool Xyt300Motor::setControlCommand(ControlCommand command)
{
  // According to protocol documentation:
  // START -> 0x0050, STOP -> 0x0052, FORWARD -> 0x0054, REVERSE -> 0x0056, EMERGENCY_STOP -> 0x0058
  // All commands write value 0x0001, and device responds with echo
  uint16_t register_address;
  switch (command) {
    case ControlCommand::START:
      register_address = REG_START;
      break;
    case ControlCommand::STOP:
      register_address = REG_STOP;  // Use proper stop register 0x0052
      break;
    case ControlCommand::FORWARD:
      register_address = REG_FORWARD;
      break;
    case ControlCommand::REVERSE:
      register_address = REG_REVERSE;
      break;
    default:
      last_error_ = "Unknown control command";
      return false;
  }
  
  // Write 0x0001 to the appropriate register (device responds with echo)
  bool result = writeRegisterNoResponse(register_address, 0x0001);
  if (!result) {
    // Log error for debugging
    std::cerr << "Xyt300Motor: Failed to set control command " << static_cast<int>(command) 
              << " (register 0x" << std::hex << register_address << std::dec 
              << "): " << last_error_ << std::endl;
  }
  return result;
}

bool Xyt300Motor::setSpeedRPM(uint16_t speed_rpm)
{
  // Input speed_rpm is output shaft RPM (after gear reduction)
  // Need to convert to PWM duty cycle (0-511) for the protocol (according to protocol documentation)
  
  // Check if speed exceeds maximum
  if (speed_rpm > max_output_rpm_) {
    last_error_ = "Speed exceeds maximum (output shaft: " + 
                  std::to_string(max_output_rpm_) + " RPM)";
    return false;
  }
  
  if (max_output_rpm_ == 0) {
    last_error_ = "Invalid configuration: max_output_rpm is zero";
    return false;
  }
  
  // Convert output shaft RPM to PWM duty cycle (0-511 according to protocol)
  // Linear mapping: duty_cycle = (speed_rpm / max_output_rpm) * 511
  // Use integer arithmetic with rounding
  uint32_t duty_cycle = (static_cast<uint32_t>(speed_rpm) * MAX_DUTY_CYCLE + max_output_rpm_ / 2) / 
                        static_cast<uint32_t>(max_output_rpm_);
  
  // Ensure duty cycle doesn't exceed maximum
  if (duty_cycle > MAX_DUTY_CYCLE) {
    duty_cycle = MAX_DUTY_CYCLE;
  }
  
  // Speed setting writes to register 0x005A
  // According to protocol: send_value = 511 - duty_cycle
  // Example: duty_cycle 0 -> send 511, duty_cycle 511 -> send 0
  uint16_t send_value = MAX_DUTY_CYCLE - static_cast<uint16_t>(duty_cycle);
  
  // Don't wait for response to improve responsiveness
  if (writeRegisterNoResponse(REG_SPEED_SETTING, send_value)) {
    speed_set_ = true;  // Mark that speed has been set
    return true;
  }
  return false;
}

bool Xyt300Motor::readStatusWord(uint16_t & status)
{
  return readRegister(REG_READ_STATUS, status);
}

bool Xyt300Motor::readCurrentSpeed(uint16_t & speed_rpm)
{
  // According to protocol documentation:
  // Function code: 0x04 (Read Input Registers)
  // Register address: 0x0020
  // Returns: pulse count (获取脉冲个数) - 2 bytes (data H and data L)
  std::vector<uint16_t> result;
  if (!modbus_client_->readInputRegisters(slave_address_, REG_READ_SPEED, 1, result)) {
    last_error_ = "Failed to read speed: " + modbus_client_->getLastError();
    return false;
  }
  
  if (result.empty()) {
    last_error_ = "Empty response when reading speed";
    return false;
  }
  
  // The register contains pulse count (获取脉冲个数)
  // According to protocol: register 0x0020 returns pulse count
  // Speed calculation formula:
  // Motor shaft RPM = (60 * pulse_frequency) / pulses_per_revolution
  // Output shaft RPM = Motor shaft RPM / gear_ratio
  // Combined: Output shaft RPM = (60 * pulse_frequency) / (pulses_per_revolution * gear_ratio)
  //
  // Note: The register might return pulse count in a specific time window
  // If it returns pulses per second (Hz), use directly
  // If it returns pulses in a different time window, adjust accordingly
  
  uint16_t pulse_count = result[0];
  
  if (pulses_per_revolution_ == 0 || gear_ratio_ == 0) {
    last_error_ = "Invalid configuration: pulses_per_revolution or gear_ratio is zero";
    return false;
  }
  
  // Assume register returns pulse frequency (pulses per second)
  // If the value seems too high (e.g., 1948 when max output RPM is 85),
  // it might be that the register returns pulses in a shorter time window
  // or returns motor shaft RPM directly
  //
  // Calculate using standard formula first
  uint32_t numerator = 60ULL * static_cast<uint32_t>(pulse_count);
  uint32_t denominator = static_cast<uint32_t>(pulses_per_revolution_) * static_cast<uint32_t>(gear_ratio_);
  uint16_t calculated_rpm = static_cast<uint16_t>(numerator / denominator);
  
  // If calculated RPM is unreasonably high (> 200), the register might return
  // motor shaft RPM directly, or pulse count in a different unit
  // Try alternative: if register value divided by gear_ratio gives reasonable result
  if (calculated_rpm > 200) {
    // Try interpreting as motor shaft RPM
    uint16_t motor_rpm = pulse_count;
    speed_rpm = motor_rpm / gear_ratio_;
    
    // If that's still too high, the register might return pulse count in 0.1 second
    // or other time window - would need device documentation to confirm
    if (speed_rpm > 200) {
      // Fall back to calculated value but log that adjustment may be needed
      speed_rpm = calculated_rpm;
    }
  } else {
    speed_rpm = calculated_rpm;
  }
  
  last_error_.clear();
  return true;
}

bool Xyt300Motor::setEnable(bool /* enable */)
{
  // Enable/disable functionality not supported by this motor
  last_error_ = "Enable/disable not supported";
  return false;
}

std::string Xyt300Motor::getLastError() const
{
  if (!last_error_.empty()) {
    return last_error_;
  }
  if (modbus_client_) {
    return modbus_client_->getLastError();
  }
  return "No error";
}

}  // namespace xyt300_motor
