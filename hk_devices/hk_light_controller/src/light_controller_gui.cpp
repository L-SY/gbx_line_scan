#include "hk_light_controller/light_controller_gui.hpp"
#include <QApplication>
#include <QMessageBox>
#include <QString>
#include <vector>

LightControllerGUI::LightControllerGUI(QWidget *parent)
  : QMainWindow(parent),
    light1_enabled_(false),
    light1_brightness_(0),
    light1_voltage_(0.0f),
    light1_current_(0.0f),
    light2_enabled_(false),
    light2_brightness_(0),
    light2_voltage_(0.0f),
    light2_current_(0.0f),
    current_trigger_source_("Unknown")
{
  setWindowTitle("光源控制器测试界面");
  setMinimumSize(600, 500);
  
  setupUI();
  setupROS2();
  
  // Timer for ROS2 spinning
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_timer_->start(10);  // 10ms interval
  
  // Timer for status updates
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &LightControllerGUI::updateStatus);
  status_timer_->start(1000);  // 1 second interval
}

LightControllerGUI::~LightControllerGUI()
{
  if (ros_timer_) {
    ros_timer_->stop();
  }
  if (status_timer_) {
    status_timer_->stop();
  }
}

void LightControllerGUI::setupUI()
{
  central_widget_ = new QWidget(this);
  setCentralWidget(central_widget_);
  
  QVBoxLayout* main_layout = new QVBoxLayout(central_widget_);
  
  // Connection status
  connection_status_label_ = new QLabel("连接状态: 未知", this);
  connection_status_label_->setStyleSheet("QLabel { font-weight: bold; font-size: 12pt; }");
  main_layout->addWidget(connection_status_label_);
  
  // Light 1 group
  light1_group_ = new QGroupBox("光源 1", this);
  QGridLayout* light1_layout = new QGridLayout(light1_group_);
  
  light1_toggle_btn_ = new QPushButton("关闭", this);
  light1_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; }");
  connect(light1_toggle_btn_, &QPushButton::clicked, this, &LightControllerGUI::onLight1Toggle);
  light1_layout->addWidget(light1_toggle_btn_, 0, 0, 1, 2);
  
  light1_status_label_ = new QLabel("状态: 关闭", this);
  light1_layout->addWidget(light1_status_label_, 1, 0);
  
  light1_brightness_label_ = new QLabel("亮度: 0", this);
  light1_layout->addWidget(light1_brightness_label_, 1, 1);
  
  light1_voltage_label_ = new QLabel("电压: 0.00 V", this);
  light1_layout->addWidget(light1_voltage_label_, 2, 0);
  
  light1_current_label_ = new QLabel("电流: 0.00 A", this);
  light1_layout->addWidget(light1_current_label_, 2, 1);
  
  light1_power_label_ = new QLabel("功率: 0.00 W", this);
  light1_power_label_->setStyleSheet("QLabel { font-weight: bold; color: #0066CC; }");
  light1_layout->addWidget(light1_power_label_, 3, 0, 1, 2);
  
  light1_brightness_slider_ = new QSlider(Qt::Horizontal, this);
  light1_brightness_slider_->setRange(0, 255);
  light1_brightness_slider_->setValue(0);
  connect(light1_brightness_slider_, &QSlider::valueChanged, 
          this, &LightControllerGUI::onLight1BrightnessChanged);
  light1_layout->addWidget(light1_brightness_slider_, 4, 0, 1, 2);
  
  main_layout->addWidget(light1_group_);
  
  // Light 2 group
  light2_group_ = new QGroupBox("光源 2", this);
  QGridLayout* light2_layout = new QGridLayout(light2_group_);
  
  light2_toggle_btn_ = new QPushButton("关闭", this);
  light2_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; }");
  connect(light2_toggle_btn_, &QPushButton::clicked, this, &LightControllerGUI::onLight2Toggle);
  light2_layout->addWidget(light2_toggle_btn_, 0, 0, 1, 2);
  
  light2_status_label_ = new QLabel("状态: 关闭", this);
  light2_layout->addWidget(light2_status_label_, 1, 0);
  
  light2_brightness_label_ = new QLabel("亮度: 0", this);
  light2_layout->addWidget(light2_brightness_label_, 1, 1);
  
  light2_voltage_label_ = new QLabel("电压: 0.00 V", this);
  light2_layout->addWidget(light2_voltage_label_, 2, 0);
  
  light2_current_label_ = new QLabel("电流: 0.00 A", this);
  light2_layout->addWidget(light2_current_label_, 2, 1);
  
  light2_power_label_ = new QLabel("功率: 0.00 W", this);
  light2_power_label_->setStyleSheet("QLabel { font-weight: bold; color: #0066CC; }");
  light2_layout->addWidget(light2_power_label_, 3, 0, 1, 2);
  
  light2_brightness_slider_ = new QSlider(Qt::Horizontal, this);
  light2_brightness_slider_->setRange(0, 255);
  light2_brightness_slider_->setValue(0);
  connect(light2_brightness_slider_, &QSlider::valueChanged, 
          this, &LightControllerGUI::onLight2BrightnessChanged);
  light2_layout->addWidget(light2_brightness_slider_, 4, 0, 1, 2);
  
  main_layout->addWidget(light2_group_);
  
  // Trigger source group
  trigger_group_ = new QGroupBox("触发源", this);
  QVBoxLayout* trigger_layout = new QVBoxLayout(trigger_group_);
  
  trigger_source_combo_ = new QComboBox(this);
  trigger_source_combo_->addItems({"Software", "Line0", "Line1", "Line2", "Line3"});
  connect(trigger_source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &LightControllerGUI::onTriggerSourceChanged);
  trigger_layout->addWidget(trigger_source_combo_);
  
  trigger_status_label_ = new QLabel("当前触发源: 未知", this);
  trigger_layout->addWidget(trigger_status_label_);
  
  main_layout->addWidget(trigger_group_);
  
  // Refresh button
  refresh_status_btn_ = new QPushButton("刷新状态", this);
  connect(refresh_status_btn_, &QPushButton::clicked, this, &LightControllerGUI::onRefreshStatus);
  main_layout->addWidget(refresh_status_btn_);
  
  main_layout->addStretch();
}

void LightControllerGUI::setupROS2()
{
  node_ = rclcpp::Node::make_shared("light_controller_gui");
  
  // Create publishers
  light1_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("light1/control", 10);
  light2_control_pub_ = node_->create_publisher<std_msgs::msg::Bool>("light2/control", 10);
  light1_brightness_pub_ = node_->create_publisher<std_msgs::msg::Int32>("light1/set_brightness", 10);
  light2_brightness_pub_ = node_->create_publisher<std_msgs::msg::Int32>("light2/set_brightness", 10);
  trigger_source_pub_ = node_->create_publisher<std_msgs::msg::String>("set_trigger_source", 10);
  
  // Create subscribers
  light1_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "light1/status", 10,
    std::bind(&LightControllerGUI::light1StatusCallback, this, std::placeholders::_1));
  
  light2_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "light2/status", 10,
    std::bind(&LightControllerGUI::light2StatusCallback, this, std::placeholders::_1));
  
  light1_brightness_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "light1/brightness", 10,
    std::bind(&LightControllerGUI::light1BrightnessCallback, this, std::placeholders::_1));
  
  light2_brightness_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "light2/brightness", 10,
    std::bind(&LightControllerGUI::light2BrightnessCallback, this, std::placeholders::_1));
  
  trigger_source_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "trigger_source", 10,
    std::bind(&LightControllerGUI::triggerSourceCallback, this, std::placeholders::_1));
  
  connection_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "connection_status", 10,
    std::bind(&LightControllerGUI::connectionStatusCallback, this, std::placeholders::_1));
  
  light1_voltage_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "light1/voltage", 10,
    std::bind(&LightControllerGUI::light1VoltageCallback, this, std::placeholders::_1));
  
  light1_current_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "light1/current", 10,
    std::bind(&LightControllerGUI::light1CurrentCallback, this, std::placeholders::_1));
  
  light2_voltage_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "light2/voltage", 10,
    std::bind(&LightControllerGUI::light2VoltageCallback, this, std::placeholders::_1));
  
  light2_current_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "light2/current", 10,
    std::bind(&LightControllerGUI::light2CurrentCallback, this, std::placeholders::_1));
  
  // Create service clients
  light1_service_client_ = node_->create_client<std_srvs::srv::SetBool>("light1/set_enabled");
  light2_service_client_ = node_->create_client<std_srvs::srv::SetBool>("light2/set_enabled");
  get_status_service_client_ = node_->create_client<std_srvs::srv::Trigger>("get_status");
}

void LightControllerGUI::onLight1Toggle()
{
  light1_enabled_ = !light1_enabled_;
  
  std_msgs::msg::Bool msg;
  msg.data = light1_enabled_;
  light1_control_pub_->publish(msg);
  
  updateUI();
}

void LightControllerGUI::onLight2Toggle()
{
  light2_enabled_ = !light2_enabled_;
  
  std_msgs::msg::Bool msg;
  msg.data = light2_enabled_;
  light2_control_pub_->publish(msg);
  
  updateUI();
}

void LightControllerGUI::onLight1BrightnessChanged(int value)
{
  light1_brightness_ = value;
  
  std_msgs::msg::Int32 msg;
  msg.data = value;
  light1_brightness_pub_->publish(msg);
  
  updateUI();
}

void LightControllerGUI::onLight2BrightnessChanged(int value)
{
  light2_brightness_ = value;
  
  std_msgs::msg::Int32 msg;
  msg.data = value;
  light2_brightness_pub_->publish(msg);
  
  updateUI();
}

void LightControllerGUI::onTriggerSourceChanged(int index)
{
  QStringList sources = {"Software", "Line0", "Line1", "Line2", "Line3"};
  if (index >= 0 && index < sources.size()) {
    std_msgs::msg::String msg;
    msg.data = sources[index].toStdString();
    trigger_source_pub_->publish(msg);
  }
}

void LightControllerGUI::onRefreshStatus()
{
  if (get_status_service_client_->wait_for_service(std::chrono::seconds(1))) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = get_status_service_client_->async_send_request(request);
  } else {
    QMessageBox::warning(this, "警告", "无法连接到状态服务");
  }
}

void LightControllerGUI::updateStatus()
{
  rclcpp::spin_some(node_);
  updateUI();
}

void LightControllerGUI::light1StatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  light1_enabled_ = msg->data;
  updateUI();
}

void LightControllerGUI::light2StatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  light2_enabled_ = msg->data;
  updateUI();
}

void LightControllerGUI::light1BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  light1_brightness_ = msg->data;
  // Block signals to avoid recursive updates
  light1_brightness_slider_->blockSignals(true);
  light1_brightness_slider_->setValue(light1_brightness_);
  light1_brightness_slider_->blockSignals(false);
  updateUI();
}

void LightControllerGUI::light2BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  light2_brightness_ = msg->data;
  // Block signals to avoid recursive updates
  light2_brightness_slider_->blockSignals(true);
  light2_brightness_slider_->setValue(light2_brightness_);
  light2_brightness_slider_->blockSignals(false);
  updateUI();
}

void LightControllerGUI::triggerSourceCallback(const std_msgs::msg::String::SharedPtr msg)
{
  current_trigger_source_ = msg->data;
  updateUI();
}

void LightControllerGUI::connectionStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    connection_status_label_->setText("连接状态: ✓ 已连接");
    connection_status_label_->setStyleSheet("QLabel { font-weight: bold; font-size: 12pt; color: green; }");
  } else {
    connection_status_label_->setText("连接状态: ✗ 未连接");
    connection_status_label_->setStyleSheet("QLabel { font-weight: bold; font-size: 12pt; color: red; }");
  }
}

void LightControllerGUI::light1VoltageCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  light1_voltage_ = msg->data;
  updateUI();
}

void LightControllerGUI::light1CurrentCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  light1_current_ = msg->data;
  updateUI();
}

void LightControllerGUI::light2VoltageCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  light2_voltage_ = msg->data;
  updateUI();
}

void LightControllerGUI::light2CurrentCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  light2_current_ = msg->data;
  updateUI();
}

void LightControllerGUI::updateUI()
{
  // Update light 1
  if (light1_enabled_) {
    light1_toggle_btn_->setText("开启");
    light1_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; background-color: #90EE90; }");
    light1_status_label_->setText("状态: 开启");
  } else {
    light1_toggle_btn_->setText("关闭");
    light1_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; background-color: #FFB6C1; }");
    light1_status_label_->setText("状态: 关闭");
  }
  light1_brightness_label_->setText(QString("亮度: %1").arg(light1_brightness_));
  light1_voltage_label_->setText(QString("电压: %1 V").arg(light1_voltage_, 0, 'f', 2));
  light1_current_label_->setText(QString("电流: %1 A").arg(light1_current_, 0, 'f', 2));
  float light1_power = light1_voltage_ * light1_current_;
  light1_power_label_->setText(QString("功率: %1 W").arg(light1_power, 0, 'f', 2));
  
  // Update light 2
  if (light2_enabled_) {
    light2_toggle_btn_->setText("开启");
    light2_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; background-color: #90EE90; }");
    light2_status_label_->setText("状态: 开启");
  } else {
    light2_toggle_btn_->setText("关闭");
    light2_toggle_btn_->setStyleSheet("QPushButton { font-size: 12pt; padding: 10px; background-color: #FFB6C1; }");
    light2_status_label_->setText("状态: 关闭");
  }
  light2_brightness_label_->setText(QString("亮度: %1").arg(light2_brightness_));
  light2_voltage_label_->setText(QString("电压: %1 V").arg(light2_voltage_, 0, 'f', 2));
  light2_current_label_->setText(QString("电流: %1 A").arg(light2_current_, 0, 'f', 2));
  float light2_power = light2_voltage_ * light2_current_;
  light2_power_label_->setText(QString("功率: %1 W").arg(light2_power, 0, 'f', 2));
  
  // Update trigger source
  trigger_status_label_->setText(QString("当前触发源: %1").arg(QString::fromStdString(current_trigger_source_)));
}

