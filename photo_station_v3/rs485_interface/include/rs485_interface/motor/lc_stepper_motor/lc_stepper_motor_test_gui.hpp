#ifndef LC_STEPPER_MOTOR_TEST_GUI_HPP
#define LC_STEPPER_MOTOR_TEST_GUI_HPP

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
#include <memory>
#include <string>

namespace rs485_interface
{
class RS485Client;
class LcStepperMotor;
}

class LcStepperMotorTestGUI : public QWidget
{
  Q_OBJECT

public:
  explicit LcStepperMotorTestGUI(QWidget * parent = nullptr);
  ~LcStepperMotorTestGUI();

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onStartClicked();
  void onStopClicked();
  void onPositionChanged(double value);
  void onSpeedChanged(double value);

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

  QPushButton * start_btn_;
  QPushButton * stop_btn_;
  QDoubleSpinBox * position_spin_;
  QLabel * position_label_;
  QDoubleSpinBox * speed_spin_;
  QLabel * speed_label_;

  QLabel * status_label_;
  QLabel * error_label_;

  // Motor components
  std::shared_ptr<rs485_interface::RS485Client> rs485_client_;
  std::shared_ptr<rs485_interface::LcStepperMotor> motor_;

  bool is_connected_;
};

#endif  // LC_STEPPER_MOTOR_TEST_GUI_HPP

