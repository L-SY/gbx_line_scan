//
// ROS2 Control Hardware Plugin - Bridges actuator interface to ros2_control
//

#include <memory>
#include <vector>
#include <string>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "actuator_interface/actuator_interface.h"

namespace actuator_interface {

class Ros2ControlHw : public hardware_interface::SystemInterface {
public:
    Ros2ControlHw() = default;
    ~Ros2ControlHw() override = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Extract config_file parameter from URDF
        std::string config_file = "";
        for (const auto& param : info_.hardware_parameters) {
            if (param.first == "config_file") {
                config_file = param.second;
                break;
            }
        }

        if (config_file.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2ControlHardwareInterface"), 
                        "config_file parameter not found in URDF hardware parameters");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Ros2ControlHardwareInterface"), 
                   "Using config file: %s", config_file.c_str());

        // Create ROS2 node for hardware interface
        auto node = std::make_shared<rclcpp::Node>("motor_drive_hardware");
        
        // Set config_file parameter on the node so ActuatorInterface can read it
        node->declare_parameter("config_file", config_file);
        
        // Initialize hardware interface
        hw_interface_ = std::make_unique<ActuatorInterface>();
        
        if (!hw_interface_->init(node)) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Failed to initialize hardware interface");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Get joint names from URDF (info_.joints comes from ros2_control tag in URDF)
        joint_names_.clear();
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            joint_names_.push_back(joint.name);
        }

        // Pass joint names to hardware interface (using convention: joint1 -> motor1)
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            std::string actuator_name = "motor" + std::to_string(i + 1);  // joint1->motor1, joint2->motor2
            hw_interface_->addJoint(joint_names_[i], actuator_name);
        }

        RCLCPP_INFO(rclcpp::get_logger("Ros2ControlHardwareInterface"), 
                   "Successfully initialized with %zu joints from URDF", joint_names_.size());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!hw_interface_->configure()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Failed to configure hardware interface");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Hardware interface configured");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!hw_interface_->activate()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Failed to activate hardware interface");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Hardware interface activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        if (!hw_interface_->deactivate()) {
            RCLCPP_ERROR(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Failed to deactivate hardware interface");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Ros2ControlHardwareInterface"), "Hardware interface deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        auto position_ptrs = hw_interface_->getJointPositionPtrs();
        auto velocity_ptrs = hw_interface_->getJointVelocityPtrs();
        auto effort_ptrs = hw_interface_->getJointEffortPtrs();

        for (size_t i = 0; i < joint_names_.size(); i++) {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, position_ptrs[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, velocity_ptrs[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_EFFORT, effort_ptrs[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        auto effort_cmd_ptrs = hw_interface_->getJointEffortCommandPtrs();
        auto position_cmd_ptrs = hw_interface_->getJointPositionCommandPtrs();

        for (size_t i = 0; i < joint_names_.size(); i++) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_EFFORT, effort_cmd_ptrs[i]));
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, position_cmd_ptrs[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        hw_interface_->read(time, period);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override {
        hw_interface_->write(time, period);
        return hardware_interface::return_type::OK;
    }

private:
    std::unique_ptr<ActuatorInterface> hw_interface_;
    std::vector<std::string> joint_names_;
};

} // namespace actuator_interface

PLUGINLIB_EXPORT_CLASS(actuator_interface::Ros2ControlHw, hardware_interface::SystemInterface) 