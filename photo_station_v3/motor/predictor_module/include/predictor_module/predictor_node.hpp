#ifndef PREDICTOR_MODULE__PREDICTOR_NODE_HPP_
#define PREDICTOR_MODULE__PREDICTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rs485_interface/motor/lc_servo_motor/lc_servo_motor_multi.hpp"

#include <memory>
#include <string>
#include <mutex>

namespace predictor_module
{

/**
 * @brief Predictor Node for controlling dual LC servo motors
 * 
 * This node manages two LC servo motors connected via RS485:
 * - Front motor (ID: 1)
 * - Rear motor (ID: 2)
 * 
 * Topics:
 * - Subscribed:
 *   - /predictor/front_motor/cmd_vel (std_msgs/Float64): Target speed for front motor (RPM)
 *   - /predictor/rear_motor/cmd_vel (std_msgs/Float64): Target speed for rear motor (RPM)
 *   - /predictor/front_motor/enable (std_msgs/Bool): Enable/disable front motor
 *   - /predictor/rear_motor/enable (std_msgs/Bool): Enable/disable rear motor
 * 
 * - Published:
 *   - /predictor/front_motor/velocity (std_msgs/Float64): Current speed of front motor (RPM)
 *   - /predictor/rear_motor/velocity (std_msgs/Float64): Current speed of rear motor (RPM)
 */
class PredictorNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit PredictorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~PredictorNode();

private:
  /**
   * @brief Initialize motors
   * @return true if successful
   */
  bool initializeMotors();

  /**
   * @brief Shutdown motors safely
   */
  void shutdownMotors();

  /**
   * @brief Callback for front motor speed command
   * @param msg Speed command message (RPM)
   */
  void frontMotorCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback for rear motor speed command
   * @param msg Speed command message (RPM)
   */
  void rearMotorCmdCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Callback for front motor enable command
   * @param msg Enable command message
   */
  void frontMotorEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Callback for rear motor enable command
   * @param msg Enable command message
   */
  void rearMotorEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief Timer callback for publishing motor velocities
   */
  void publishVelocityCallback();

  /**
   * @brief Set motor speed with direction handling
   * @param motor_id Motor ID (1 for front, 2 for rear)
   * @param speed_rpm Speed in RPM (negative for reverse)
   * @return true if successful
   */
  bool setMotorSpeed(uint8_t motor_id, double speed_rpm);

  // Motor manager
  std::unique_ptr<rs485_interface::LcServoMotorMulti> motor_manager_;

  // Mutex for thread-safe motor access
  std::mutex motor_mutex_;

  // Parameters
  std::string device_path_;
  int baud_rate_;
  uint8_t front_motor_id_;
  uint8_t rear_motor_id_;
  double publish_rate_;

  // Motor states
  bool front_motor_enabled_;
  bool rear_motor_enabled_;
  double front_motor_target_speed_;
  double rear_motor_target_speed_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_motor_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rear_motor_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_motor_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rear_motor_enable_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_motor_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_motor_vel_pub_;

  // Timer for velocity publishing
  rclcpp::TimerBase::SharedPtr velocity_timer_;
};

}  // namespace predictor_module

#endif  // PREDICTOR_MODULE__PREDICTOR_NODE_HPP_
