#ifndef MOTOR_CONTROL_WIDGET_HPP
#define MOTOR_CONTROL_WIDGET_HPP

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QTimer>
#include <memory>
#include <string>

namespace rs485_interface
{
class RS485Client;
class RS485DeviceBase;
}

/**
 * @brief Base widget for motor control that can be embedded in the unified GUI
 * 
 * This abstract class provides the common interface and UI elements for all
 * motor control widgets. Derived classes implement motor-specific controls.
 */
class MotorControlWidget : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Motor type for widget identification
   */
  enum class MotorType
  {
    ZD_MOTOR,
    LC_SERVO_MOTOR,
    LC_STEPPER_MOTOR
  };

  explicit MotorControlWidget(
    const QString & motor_name,
    MotorType type,
    QWidget * parent = nullptr);
  virtual ~MotorControlWidget() = default;

  /**
   * @brief Get motor name
   */
  QString getMotorName() const { return motor_name_; }

  /**
   * @brief Get motor type
   */
  MotorType getMotorType() const { return motor_type_; }

  /**
   * @brief Check if connected to a motor
   */
  virtual bool isConnected() const = 0;

  /**
   * @brief Get the RS485 device
   */
  virtual std::shared_ptr<rs485_interface::RS485DeviceBase> getDevice() const = 0;

  /**
   * @brief Set the RS485 client for this motor
   * @param client Shared pointer to RS485 client
   * @param slave_address Motor slave address
   */
  virtual void setRS485Client(
    std::shared_ptr<rs485_interface::RS485Client> client,
    uint8_t slave_address) = 0;

signals:
  /**
   * @brief Emitted when connection status changes
   */
  void connectionStatusChanged(bool connected);

  /**
   * @brief Emitted when an error occurs
   */
  void errorOccurred(const QString & error);

  /**
   * @brief Emitted when status message needs to be shown
   */
  void statusMessage(const QString & message);

protected:
  /**
   * @brief Update UI to reflect connection status
   */
  virtual void updateConnectionStatus(bool connected) = 0;

  /**
   * @brief Log a message (emit statusMessage signal)
   */
  void logMessage(const std::string & message);

  QString motor_name_;
  MotorType motor_type_;
};

#endif  // MOTOR_CONTROL_WIDGET_HPP

