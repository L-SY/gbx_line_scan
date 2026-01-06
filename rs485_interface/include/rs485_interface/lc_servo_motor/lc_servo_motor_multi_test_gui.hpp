#ifndef LC_SERVO_MOTOR_MULTI_TEST_GUI_HPP
#define LC_SERVO_MOTOR_MULTI_TEST_GUI_HPP

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
#include <QScrollArea>
#include <QTabWidget>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cstdint>

// Forward declarations
namespace rs485_interface
{
class LcServoMotorMulti;
class LcServoMotor;
}

// For Qt signals, we need to use int instead of enum class
// The enum will be converted in the implementation
using MotorDirection = int;

/**
 * @brief Widget for controlling a single motor in multi-motor GUI
 */
class SingleMotorControlWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SingleMotorControlWidget(
    uint8_t address,
    const std::string & name,
    QWidget * parent = nullptr);

  void setMotor(std::shared_ptr<rs485_interface::LcServoMotor> motor);
  void updateCurrentSpeed();
  void setEnabled(bool enabled);

signals:
  void speedChanged(uint8_t address, double speed_rpm);
  void directionChanged(uint8_t address, MotorDirection direction);
  void enableChanged(uint8_t address, bool enable);

private slots:
  void onInitializeClicked();
  void onEnableClicked();
  void onDisableClicked();
  void onForwardClicked();
  void onReverseClicked();
  void onStopClicked();
  void onSpeedChanged(double value);
  void onUpdateSpeed();

private:
  void setupUI();

  uint8_t address_;
  std::string name_;
  std::shared_ptr<rs485_interface::LcServoMotor> motor_;

  QPushButton * init_btn_;
  QPushButton * enable_btn_;
  QPushButton * disable_btn_;
  QPushButton * forward_btn_;
  QPushButton * reverse_btn_;
  QPushButton * stop_btn_;
  QDoubleSpinBox * speed_spin_;
  QLabel * speed_label_;
  QLabel * current_speed_label_;
  QLabel * status_label_;
  QLabel * error_label_;

  QTimer * speed_update_timer_;
  bool is_initialized_;
};

/**
 * @brief Multi-motor test GUI for LC Servo Motors
 */
class LcServoMotorMultiTestGUI : public QWidget
{
  Q_OBJECT

public:
  explicit LcServoMotorMultiTestGUI(QWidget * parent = nullptr);
  ~LcServoMotorMultiTestGUI();

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onAddMotorClicked();
  void onRemoveMotorClicked();
  void onInitializeAllClicked();
  void onEnableAllClicked();
  void onDisableAllClicked();
  void onForwardAllClicked();
  void onReverseAllClicked();
  void onStopAllClicked();
  void onSpeedAllChanged(double value);
  void onUpdateAllSpeeds();

private:
  void setupUI();
  void updateConnectionStatus(bool connected);
  void updateMotorList();
  void logMessage(const std::string & message);
  SingleMotorControlWidget * createMotorWidget(uint8_t address, const std::string & name);

  // UI components
  QVBoxLayout * main_layout_;
  QGroupBox * connection_group_;
  QGroupBox * motor_management_group_;
  QGroupBox * global_control_group_;
  QGroupBox * status_group_;

  QLineEdit * port_edit_;
  QComboBox * baud_combo_;
  QPushButton * connect_btn_;
  QPushButton * disconnect_btn_;

  QSpinBox * add_motor_address_spin_;
  QLineEdit * add_motor_name_edit_;
  QPushButton * add_motor_btn_;
  QComboBox * remove_motor_combo_;
  QPushButton * remove_motor_btn_;

  QPushButton * init_all_btn_;
  QPushButton * enable_all_btn_;
  QPushButton * disable_all_btn_;
  QPushButton * forward_all_btn_;
  QPushButton * reverse_all_btn_;
  QPushButton * stop_all_btn_;
  QDoubleSpinBox * speed_all_spin_;
  QLabel * speed_all_label_;

  QLabel * status_label_;
  QLabel * error_label_;

  QTabWidget * motor_tabs_;

  // Motor components
  std::shared_ptr<rs485_interface::LcServoMotorMulti> motor_multi_;
  std::map<uint8_t, SingleMotorControlWidget *> motor_widgets_;

  // Timer for reading current speeds
  QTimer * speed_update_timer_;

  bool is_connected_;
};

#endif  // LC_SERVO_MOTOR_MULTI_TEST_GUI_HPP

