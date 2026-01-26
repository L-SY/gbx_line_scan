#ifndef XYT300_MOTOR_GUI_HPP
#define XYT300_MOTOR_GUI_HPP

#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QStatusBar>
#include <QTimer>
#include <QMessageBox>
#include <QDialog>
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QThread>
#include <QProgressDialog>
#include <memory>
#include <string>
#include <vector>

namespace xyt300_motor
{
class ModbusClient;
class Xyt300Motor;
}

/**
 * @brief Dialog for changing motor slave address
 */
class ChangeAddressDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ChangeAddressDialog(uint8_t current_address, QWidget * parent = nullptr);
  uint8_t getNewAddress() const;

private:
  QSpinBox * address_spin_;
};

/**
 * @brief Main GUI window for XYT300 motor control
 */
class Xyt300MotorGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit Xyt300MotorGUI(QWidget * parent = nullptr);
  ~Xyt300MotorGUI();

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onInitializeClicked();
  void onForwardClicked();
  void onReverseClicked();
  void onStopClicked();
  void onSpeedChanged(int value);
  void onChangeAddressClicked();
  void onScanIdsClicked();
  void onUpdateStatus();

private:
  void setupUI();
  void updateConnectionStatus(bool connected);
  bool connectMotor();
  void disconnectMotor();

  // UI Components
  QWidget * central_widget_;
  QGroupBox * connection_group_;
  QLineEdit * device_path_edit_;
  QComboBox * baud_rate_combo_;
  QSpinBox * slave_address_spin_;
  QPushButton * connect_btn_;
  QPushButton * disconnect_btn_;
  QPushButton * init_btn_;
  QPushButton * change_address_btn_;
  QPushButton * scan_ids_btn_;

  QGroupBox * control_group_;
  QPushButton * forward_btn_;
  QPushButton * reverse_btn_;
  QPushButton * stop_btn_;
  QSlider * speed_slider_;
  QLabel * speed_value_label_;

  QGroupBox * status_group_;
  QLabel * status_label_;
  QLabel * current_speed_label_;
  QLabel * error_label_;

  QStatusBar * status_bar_;

  // Motor instances
  std::shared_ptr<xyt300_motor::ModbusClient> modbus_client_;
  std::unique_ptr<xyt300_motor::Xyt300Motor> motor_;

  // State
  bool is_connected_;
  bool is_initialized_;

  // Timer for status updates
  QTimer * status_timer_;
  
  // Scan related
  QThread * scan_thread_;
  QProgressDialog * scan_progress_;
  
  // Helper method for scan results
  void onScanIdsFinished(const std::vector<uint8_t> & found_ids, bool success);
};

#endif  // XYT300_MOTOR_GUI_HPP
