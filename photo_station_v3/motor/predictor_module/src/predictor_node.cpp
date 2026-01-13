#include "predictor_module/predictor_node.hpp"

#include <chrono>
#include <cmath>

namespace predictor_module
{

PredictorNode::PredictorNode(const rclcpp::NodeOptions & options)
: Node("predictor_node", options),
  front_motor_enabled_(false),
  rear_motor_enabled_(false),
  front_motor_target_speed_(0.0),
  rear_motor_target_speed_(0.0)
{
  // Declare parameters
  this->declare_parameter<std::string>("device_path", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 19200);
  this->declare_parameter<int>("front_motor_id", 1);
  this->declare_parameter<int>("rear_motor_id", 2);
  this->declare_parameter<double>("publish_rate", 10.0);

  // Get parameters
  device_path_ = this->get_parameter("device_path").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  front_motor_id_ = static_cast<uint8_t>(this->get_parameter("front_motor_id").as_int());
  rear_motor_id_ = static_cast<uint8_t>(this->get_parameter("rear_motor_id").as_int());
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Predictor Node initializing...");
  RCLCPP_INFO(this->get_logger(), "  Device path: %s", device_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Baud rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "  Front motor ID: %d", front_motor_id_);
  RCLCPP_INFO(this->get_logger(), "  Rear motor ID: %d", rear_motor_id_);

  // Initialize motors
  if (!initializeMotors()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize motors!");
    throw std::runtime_error("Motor initialization failed");
  }

  // Create subscribers for motor commands
  front_motor_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/predictor/front_motor/cmd_vel", 10,
    std::bind(&PredictorNode::frontMotorCmdCallback, this, std::placeholders::_1));

  rear_motor_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/predictor/rear_motor/cmd_vel", 10,
    std::bind(&PredictorNode::rearMotorCmdCallback, this, std::placeholders::_1));

  front_motor_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/predictor/front_motor/enable", 10,
    std::bind(&PredictorNode::frontMotorEnableCallback, this, std::placeholders::_1));

  rear_motor_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/predictor/rear_motor/enable", 10,
    std::bind(&PredictorNode::rearMotorEnableCallback, this, std::placeholders::_1));

  // Create publishers for motor velocities
  front_motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/predictor/front_motor/velocity", 10);

  rear_motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/predictor/rear_motor/velocity", 10);

  // Create timer for velocity publishing
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  velocity_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&PredictorNode::publishVelocityCallback, this));

  RCLCPP_INFO(this->get_logger(), "Predictor Node initialized successfully!");
  RCLCPP_INFO(this->get_logger(), "Subscribed topics:");
  RCLCPP_INFO(this->get_logger(), "  /predictor/front_motor/cmd_vel (Float64, RPM)");
  RCLCPP_INFO(this->get_logger(), "  /predictor/rear_motor/cmd_vel (Float64, RPM)");
  RCLCPP_INFO(this->get_logger(), "  /predictor/front_motor/enable (Bool)");
  RCLCPP_INFO(this->get_logger(), "  /predictor/rear_motor/enable (Bool)");
  RCLCPP_INFO(this->get_logger(), "Published topics:");
  RCLCPP_INFO(this->get_logger(), "  /predictor/front_motor/velocity (Float64, RPM)");
  RCLCPP_INFO(this->get_logger(), "  /predictor/rear_motor/velocity (Float64, RPM)");
}

PredictorNode::~PredictorNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Predictor Node...");
  shutdownMotors();
}

bool PredictorNode::initializeMotors()
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  // Determine baud rate enum
  rs485_interface::RS485ClientServo::BaudRate baud;
  switch (baud_rate_) {
    case 9600:
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_9600;
      break;
    case 19200:
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_19200;
      break;
    case 38400:
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_38400;
      break;
    case 57600:
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_57600;
      break;
    case 115200:
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_115200;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, using 19200", baud_rate_);
      baud = rs485_interface::RS485ClientServo::BaudRate::BAUD_19200;
      break;
  }

  // Create motor manager
  motor_manager_ = std::make_unique<rs485_interface::LcServoMotorMulti>(
    device_path_,
    baud,
    rs485_interface::RS485ClientServo::Parity::EVEN,
    2000  // timeout_ms
  );

  // Add front motor
  if (!motor_manager_->addMotor(front_motor_id_, "Front Motor")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add front motor (ID: %d): %s",
      front_motor_id_, motor_manager_->getLastError().c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Added front motor (ID: %d)", front_motor_id_);

  // Add rear motor
  if (!motor_manager_->addMotor(rear_motor_id_, "Rear Motor")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add rear motor (ID: %d): %s",
      rear_motor_id_, motor_manager_->getLastError().c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Added rear motor (ID: %d)", rear_motor_id_);

  // Open connection
  if (!motor_manager_->open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open RS485 connection: %s",
      motor_manager_->getLastError().c_str());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "RS485 connection opened");

  // Initialize all motors for speed control
  auto init_results = motor_manager_->initializeAllMotors();
  for (const auto & result : init_results) {
    if (result.second) {
      RCLCPP_INFO(this->get_logger(), "Motor %d initialized for speed control", result.first);
    } else {
      RCLCPP_WARN(this->get_logger(), "Motor %d initialization warning (may still work)", result.first);
    }
  }

  return true;
}

void PredictorNode::shutdownMotors()
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  if (motor_manager_) {
    RCLCPP_INFO(this->get_logger(), "Stopping all motors...");
    motor_manager_->stopAllMotors();
    motor_manager_->disableAllMotors();
    motor_manager_->close();
    motor_manager_.reset();
  }
}

void PredictorNode::frontMotorCmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  front_motor_target_speed_ = msg->data;
  
  if (!front_motor_enabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Front motor not enabled, ignoring speed command");
    return;
  }

  if (setMotorSpeed(front_motor_id_, msg->data)) {
    RCLCPP_DEBUG(this->get_logger(), "Front motor speed set to %.2f RPM", msg->data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set front motor speed");
  }
}

void PredictorNode::rearMotorCmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  rear_motor_target_speed_ = msg->data;
  
  if (!rear_motor_enabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Rear motor not enabled, ignoring speed command");
    return;
  }

  if (setMotorSpeed(rear_motor_id_, msg->data)) {
    RCLCPP_DEBUG(this->get_logger(), "Rear motor speed set to %.2f RPM", msg->data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set rear motor speed");
  }
}

void PredictorNode::frontMotorEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  if (!motor_manager_) {
    RCLCPP_ERROR(this->get_logger(), "Motor manager not initialized");
    return;
  }

  auto motor = motor_manager_->getMotor(front_motor_id_);
  if (!motor) {
    RCLCPP_ERROR(this->get_logger(), "Front motor not found");
    return;
  }

  if (msg->data) {
    // Enable motor
    if (motor->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE)) {
      front_motor_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "Front motor enabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable front motor: %s", motor->getLastError().c_str());
    }
  } else {
    // Disable motor (stop first)
    motor->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
    if (motor->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE)) {
      front_motor_enabled_ = false;
      RCLCPP_INFO(this->get_logger(), "Front motor disabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to disable front motor: %s", motor->getLastError().c_str());
    }
  }
}

void PredictorNode::rearMotorEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  if (!motor_manager_) {
    RCLCPP_ERROR(this->get_logger(), "Motor manager not initialized");
    return;
  }

  auto motor = motor_manager_->getMotor(rear_motor_id_);
  if (!motor) {
    RCLCPP_ERROR(this->get_logger(), "Rear motor not found");
    return;
  }

  if (msg->data) {
    // Enable motor
    if (motor->setEnable(rs485_interface::LcServoMotor::EnableState::ENABLE)) {
      rear_motor_enabled_ = true;
      RCLCPP_INFO(this->get_logger(), "Rear motor enabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable rear motor: %s", motor->getLastError().c_str());
    }
  } else {
    // Disable motor (stop first)
    motor->setDirection(rs485_interface::LcServoMotor::Direction::STOP);
    if (motor->setEnable(rs485_interface::LcServoMotor::EnableState::DISABLE)) {
      rear_motor_enabled_ = false;
      RCLCPP_INFO(this->get_logger(), "Rear motor disabled");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to disable rear motor: %s", motor->getLastError().c_str());
    }
  }
}

void PredictorNode::publishVelocityCallback()
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  if (!motor_manager_ || !motor_manager_->isOpen()) {
    return;
  }

  // Read and publish front motor velocity
  auto front_motor = motor_manager_->getMotor(front_motor_id_);
  if (front_motor) {
    double front_speed = 0.0;
    if (front_motor->readCurrentSpeed(front_speed)) {
      auto msg = std_msgs::msg::Float64();
      msg.data = front_speed;
      front_motor_vel_pub_->publish(msg);
    }
  }

  // Read and publish rear motor velocity
  auto rear_motor = motor_manager_->getMotor(rear_motor_id_);
  if (rear_motor) {
    double rear_speed = 0.0;
    if (rear_motor->readCurrentSpeed(rear_speed)) {
      auto msg = std_msgs::msg::Float64();
      msg.data = rear_speed;
      rear_motor_vel_pub_->publish(msg);
    }
  }
}

bool PredictorNode::setMotorSpeed(uint8_t motor_id, double speed_rpm)
{
  std::lock_guard<std::mutex> lock(motor_mutex_);

  if (!motor_manager_) {
    RCLCPP_ERROR(this->get_logger(), "Motor manager not initialized");
    return false;
  }

  auto motor = motor_manager_->getMotor(motor_id);
  if (!motor) {
    RCLCPP_ERROR(this->get_logger(), "Motor %d not found", motor_id);
    return false;
  }

  // Determine direction based on speed sign
  rs485_interface::LcServoMotor::Direction direction;
  double abs_speed = std::abs(speed_rpm);

  if (abs_speed < 0.01) {
    // Stop if speed is essentially zero
    direction = rs485_interface::LcServoMotor::Direction::STOP;
  } else if (speed_rpm > 0) {
    direction = rs485_interface::LcServoMotor::Direction::FORWARD;
  } else {
    direction = rs485_interface::LcServoMotor::Direction::REVERSE;
  }

  // Set direction
  if (!motor->setDirection(direction)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set motor %d direction: %s",
      motor_id, motor->getLastError().c_str());
    return false;
  }

  // Set speed (absolute value)
  if (direction != rs485_interface::LcServoMotor::Direction::STOP) {
    if (!motor->setSpeedRPM(abs_speed)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set motor %d speed: %s",
        motor_id, motor->getLastError().c_str());
      return false;
    }
  }

  return true;
}

}  // namespace predictor_module

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<predictor_module::PredictorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("predictor_node"), "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
