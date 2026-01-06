#ifndef LC_SERVO_MOTOR_TEST_GUI_HPP
#define LC_SERVO_MOTOR_TEST_GUI_HPP

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
class RS485ClientServo;
class LcServoMotor;
}

class LcServoMotorTestGUI : public QWidget
{
  Q_OBJECT

public:
  explicit LcServoMotorTestGUI(QWidget * parent = nullptr);
  ~LcServoMotorTestGUI();

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
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
  void updateConnectionStatus(bool connected);
  void logMessage(const std::string & message);

  // UI components
  QVBoxLayout * main_layout_;
  QGroupBox * connection_group_;
  QGroupBox * control_group_;
  QGroupBox * status_group_;

  QLineEdit * port_edit_;
  QComboBox * baud_combo_;
  QSpinBox * address_spin_;
  QPushButton * connect_btn_;
  QPushButton * disconnect_btn_;

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

  // Motor components
  std::shared_ptr<rs485_interface::RS485ClientServo> rs485_client_;
  std::shared_ptr<rs485_interface::LcServoMotor> motor_;

  // Timer for reading current speed
  QTimer * speed_update_timer_;

  bool is_connected_;
};

#endif  // LC_SERVO_MOTOR_TEST_GUI_HPP

