#include "rs485_interface/distance_sensor.hpp"

namespace rs485_interface
{

DistanceSensor::DistanceSensor(
  std::shared_ptr<ModbusClient> modbus_client,
  uint8_t slave_address)
: DeviceBase(modbus_client, slave_address)
{
}

bool DistanceSensor::initialize()
{
  if (!modbus_client_) {
    last_error_ = "MODBUS client is null";
    return false;
  }

  if (!modbus_client_->isOpen()) {
    last_error_ = "MODBUS client is not open";
    return false;
  }

  // Try to read distance as a test
  double test_distance;
  if (!readDistance(test_distance)) {
    last_error_ = "Failed to initialize: " + getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

std::string DistanceSensor::getDeviceType() const
{
  return "SGF Series Photoelectric Distance Sensor";
}

bool DistanceSensor::readDistance(double & distance)
{
  uint32_t distance_um;
  if (!readDistanceMicrometers(distance_um)) {
    return false;
  }

  distance = distance_um / 1000.0;  // Convert micrometers to millimeters
  return true;
}

bool DistanceSensor::readDistanceMicrometers(uint32_t & distance)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_DISTANCE,
        2,
        registers))
  {
    last_error_ = "Failed to read distance: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.size() < 2) {
    last_error_ = "Invalid response: insufficient registers";
    return false;
  }

  // Combine two 16-bit registers into 32-bit value (high word first)
  distance = (static_cast<uint32_t>(registers[0]) << 16) | registers[1];

  // Check for invalid measurement value (0x7FFFFFFF = 2147483647)
  // This value is returned when sensor cannot measure (weak reflection, out of range, etc.)
  if (distance == 0x7FFFFFFF) {
    last_error_ = "Invalid measurement: weak reflection or out of range";
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readWorkingMode(WorkingMode & mode)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_WORKING_MODE,
        1,
        registers))
  {
    last_error_ = "Failed to read working mode: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  mode = static_cast<WorkingMode>(registers[0]);
  last_error_.clear();
  return true;
}

bool DistanceSensor::setWorkingMode(WorkingMode mode)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_WORKING_MODE,
        static_cast<uint16_t>(mode)))
  {
    last_error_ = "Failed to set working mode: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readSwitchMode(SwitchMode & mode)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_SWITCH_MODE,
        1,
        registers))
  {
    last_error_ = "Failed to read switch mode: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  mode = static_cast<SwitchMode>(registers[0]);
  last_error_.clear();
  return true;
}

bool DistanceSensor::setSwitchMode(SwitchMode mode)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_SWITCH_MODE,
        static_cast<uint16_t>(mode)))
  {
    last_error_ = "Failed to set switch mode: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readDetectionOutput(DetectionOutput & mode)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_DETECTION_OUTPUT,
        1,
        registers))
  {
    last_error_ = "Failed to read detection output: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  mode = static_cast<DetectionOutput>(registers[0]);
  last_error_.clear();
  return true;
}

bool DistanceSensor::setDetectionOutput(DetectionOutput mode)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_DETECTION_OUTPUT,
        static_cast<uint16_t>(mode)))
  {
    last_error_ = "Failed to set detection output: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readAnalogSelection(AnalogSelection & selection)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_ANALOG_SELECTION,
        1,
        registers))
  {
    last_error_ = "Failed to read analog selection: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  selection = static_cast<AnalogSelection>(registers[0]);
  last_error_.clear();
  return true;
}

bool DistanceSensor::setAnalogSelection(AnalogSelection selection)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_ANALOG_SELECTION,
        static_cast<uint16_t>(selection)))
  {
    last_error_ = "Failed to set analog selection: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readDifferential(uint16_t & differential)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_DIFFERENTIAL,
        1,
        registers))
  {
    last_error_ = "Failed to read differential: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  differential = registers[0];
  last_error_.clear();
  return true;
}

bool DistanceSensor::setDifferential(uint16_t differential)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_DIFFERENTIAL,
        differential))
  {
    last_error_ = "Failed to set differential: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readDisplayMode(DisplayMode & mode)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_DISPLAY_MODE,
        1,
        registers))
  {
    last_error_ = "Failed to read display mode: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  mode = static_cast<DisplayMode>(registers[0]);
  last_error_.clear();
  return true;
}

bool DistanceSensor::setDisplayMode(DisplayMode mode)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_DISPLAY_MODE,
        static_cast<uint16_t>(mode)))
  {
    last_error_ = "Failed to set display mode: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::readBaudRate(uint16_t & baud_rate)
{
  std::vector<uint16_t> registers;
  if (!modbus_client_->readHoldingRegisters(
        slave_address_,
        REG_BAUD_RATE,
        1,
        registers))
  {
    last_error_ = "Failed to read baud rate: " + modbus_client_->getLastError();
    return false;
  }

  if (registers.empty()) {
    last_error_ = "Invalid response: no registers";
    return false;
  }

  baud_rate = registers[0];
  last_error_.clear();
  return true;
}

bool DistanceSensor::setBaudRate(uint16_t baud_rate_index)
{
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_BAUD_RATE,
        baud_rate_index))
  {
    last_error_ = "Failed to set baud rate: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::zero()
{
  // Write 1 to zero register
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_OPERATION_ZERO,
        1))
  {
    last_error_ = "Failed to perform zeroing: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::teach()
{
  // Write 1 to teach register
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_OPERATION_TEACH,
        1))
  {
    last_error_ = "Failed to perform teaching: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::stopLaser()
{
  // Write 1 to stop register
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_OPERATION_STOP,
        1))
  {
    last_error_ = "Failed to stop laser: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

bool DistanceSensor::reset()
{
  // Write 1 to reset register
  if (!modbus_client_->writeSingleRegister(
        slave_address_,
        REG_OPERATION_RESET,
        1))
  {
    last_error_ = "Failed to reset: " + modbus_client_->getLastError();
    return false;
  }

  last_error_.clear();
  return true;
}

}  // namespace rs485_interface

