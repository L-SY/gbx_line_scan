#ifndef UNIFIED_MOTOR_GUI_HPP
#define UNIFIED_MOTOR_GUI_HPP

#include <QMainWindow>
#include <QWidget>
#include <QDialog>
#include <QPushButton>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QTabWidget>
#include <QListWidget>
#include <QSplitter>
#include <QStatusBar>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QTimer>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace rs485_interface
{
class RS485Client;
class RS485DeviceBase;
class ZdMotor;
class LcServoMotor;
class LcStepperMotor;
}

/**
 * @brief Configuration for a motor connection
 */
struct MotorConfig
{
  enum class MotorType { ZD_MOTOR, LC_SERVO, LC_STEPPER };
  
  QString name;
  MotorType type;
  QString port;
  int baud_rate;
  int parity;  // 0=NONE, 1=EVEN, 2=ODD
  uint8_t slave_address;
};

/**
 * @brief Widget for controlling a single motor in the unified GUI
 */
class SingleMotorWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SingleMotorWidget(const MotorConfig & config, QWidget * parent = nullptr);
  ~SingleMotorWidget();

  bool connectMotor();
  void disconnectMotor();
  bool isConnected() const { return is_connected_; }
  
  /**
   * @brief Stop the motor (public interface for emergency stop)
   */
  void stopMotor();
  
  QString getMotorName() const { return config_.name; }
  MotorConfig::MotorType getMotorType() const { return config_.type; }
  std::shared_ptr<rs485_interface::RS485DeviceBase> getDevice() const;

signals:
  void connectionChanged(bool connected);
  void statusMessage(const QString & message);
  void errorMessage(const QString & error);

private slots:
  void onConnectClicked();
  void onDisconnectClicked();
  void onInitializeClicked();
  void onForwardClicked();
  void onReverseClicked();
  void onStopClicked();
  void onSpeedChanged(double value);
  void onPositionChanged(double value);  // For stepper - send immediately on change
  void onStartClicked();  // For stepper - enable motor
  void onUpdateStatus();

private:
  void setupUI();
  void setupZdMotorControls(QVBoxLayout * layout);
  void setupServoMotorControls(QVBoxLayout * layout);
  void setupStepperMotorControls(QVBoxLayout * layout);
  void updateConnectionStatus(bool connected);

  MotorConfig config_;
  bool is_connected_;
  bool is_busy_;  // Prevent command conflicts

  // Common UI elements
  QLabel * status_label_;
  QLabel * error_label_;
  QPushButton * connect_btn_;
  QPushButton * disconnect_btn_;
  QPushButton * init_btn_;
  QPushButton * forward_btn_;
  QPushButton * reverse_btn_;
  QPushButton * stop_btn_;
  QDoubleSpinBox * speed_spin_;
  QLabel * current_speed_label_;
  
  // Stepper-specific controls
  QDoubleSpinBox * position_spin_;
  QPushButton * start_motor_btn_;  // For stepper: move button
  QPushButton * set_speed_btn_;    // For servo/ZD: set speed button
  
  QTimer * status_timer_;

  // Motor instances (only one will be used based on type)
  std::shared_ptr<rs485_interface::RS485Client> rs485_client_;
  std::shared_ptr<rs485_interface::ZdMotor> zd_motor_;
  std::shared_ptr<rs485_interface::LcServoMotor> servo_motor_;
  std::shared_ptr<rs485_interface::LcStepperMotor> stepper_motor_;
};

/**
 * @brief Unified GUI for managing multiple RS485 motors
 * 
 * This main window allows users to:
 * - Add/remove motors of different types (ZD, LC Servo, LC Stepper)
 * - Connect to motors on different RS485 buses (different serial ports)
 * - Control all motors from a single interface
 * - Monitor status of all connected motors
 */
class UnifiedMotorGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit UnifiedMotorGUI(QWidget * parent = nullptr);
  ~UnifiedMotorGUI();

private slots:
  void onAddMotorClicked();
  void onRemoveMotorClicked();
  void onConnectAllClicked();
  void onDisconnectAllClicked();
  void onStopAllClicked();
  void onMotorSelected(int index);
  void onMotorStatusChanged(const QString & name, bool connected);

private:
  void setupUI();
  void setupMenuBar();
  void addMotor(const MotorConfig & config);
  void removeMotor(const QString & name);
  void updateMotorList();
  void logMessage(const QString & message);

  // UI Components
  QWidget * central_widget_;
  QSplitter * main_splitter_;
  
  // Left panel - Motor list
  QGroupBox * motor_list_group_;
  QListWidget * motor_list_;
  QPushButton * add_motor_btn_;
  QPushButton * remove_motor_btn_;
  
  // Right panel - Motor control tabs
  QTabWidget * motor_tabs_;
  
  // Bottom panel - Global controls
  QGroupBox * global_control_group_;
  QPushButton * connect_all_btn_;
  QPushButton * disconnect_all_btn_;
  QPushButton * stop_all_btn_;
  
  // Status bar
  QLabel * status_label_;
  
  // Motor management
  std::map<QString, SingleMotorWidget *> motor_widgets_;
  std::vector<MotorConfig> motor_configs_;
};

/**
 * @brief Dialog for adding a new motor
 */
class AddMotorDialog : public QDialog
{
  Q_OBJECT

public:
  explicit AddMotorDialog(QWidget * parent = nullptr);
  
  MotorConfig getConfig() const;

private slots:
  void onMotorTypeChanged(int index);
  void onBrowsePortClicked();

private:
  void setupUI();
  void updateDefaultSettings();

  QLineEdit * name_edit_;
  QComboBox * type_combo_;
  QLineEdit * port_edit_;
  QPushButton * browse_port_btn_;
  QComboBox * baud_combo_;
  QComboBox * parity_combo_;
  QSpinBox * address_spin_;
  QPushButton * ok_btn_;
  QPushButton * cancel_btn_;
};

#endif  // UNIFIED_MOTOR_GUI_HPP

