#ifndef ZD_MOTOR_TEST_GUI_HPP
#define ZD_MOTOR_TEST_GUI_HPP

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QTimer>
#include <memory>
#include <string>

#include "rs485_interface/common/rs485_client.hpp"
#include "rs485_interface/motor/zd_motor/zd_motor.hpp"

class ZdMotorTestGUI : public QWidget
{
  Q_OBJECT

public:
  explicit ZdMotorTestGUI(QWidget * parent = nullptr);
  ~ZdMotorTestGUI();

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onScanIdClicked();
  void onReadIdClicked();
  void onSetIdClicked();
  void onForwardClicked();
  void onReverseClicked();
  void onStopClicked();
  void onFaultResetClicked();
  void onSpeedChanged(int value);
  void onReadStatusClicked();
  void onUpdateStatus();

private:
  void setupUI();
  void updateConnectionStatus(bool connected);
  void logMessage(const std::string & message);

  // UI components
  QVBoxLayout * main_layout_;
  QGroupBox * connection_group_;
  QGroupBox * id_management_group_;
  QGroupBox * control_group_;
  QGroupBox * status_group_;

  QLineEdit * port_edit_;
  QComboBox * baud_combo_;
  QSpinBox * address_spin_;
  QPushButton * connect_btn_;
  QPushButton * disconnect_btn_;

  QPushButton * scan_id_btn_;
  QPushButton * read_id_btn_;
  QSpinBox * set_id_spin_;
  QPushButton * set_id_btn_;
  QLabel * current_id_label_;

  QPushButton * forward_btn_;
  QPushButton * reverse_btn_;
  QPushButton * stop_btn_;
  QPushButton * fault_reset_btn_;
  QSpinBox * speed_spin_;
  QLabel * speed_label_;

  QPushButton * read_status_btn_;
  QLabel * status_word1_label_;
  QLabel * status_word2_label_;

  QLabel * status_label_;
  QLabel * error_label_;

  // Motor components
  std::shared_ptr<rs485_interface::RS485ClientServo> rs485_client_;
  std::shared_ptr<rs485_interface::ZdMotor> motor_;

  // Timer for reading status
  QTimer * status_update_timer_;

  bool is_connected_;
};

#endif  // ZD_MOTOR_TEST_GUI_HPP

