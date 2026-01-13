//
// Actuator Interface implementation for ROS2
//

#include "actuator_interface/actuator_interface.h"
#include <rclcpp/rclcpp.hpp>

namespace actuator_interface {

ActuatorInterface::ActuatorInterface()
    : logger_(rclcpp::get_logger("ActuatorInterface"))
    , configured_(false)
    , activated_(false)
{
    RCLCPP_INFO(logger_, "Initializing ActuatorInterface");
}

bool ActuatorInterface::init(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
    logger_ = node_->get_logger();
    
    RCLCPP_INFO(logger_, "Starting hardware interface initialization");
    
    // Load configuration
    if (!loadConfiguration()) {
        RCLCPP_ERROR(logger_, "Failed to load configuration");
        return false;
    }
    
    // Setup joint mapping
    if (!setupJointMapping()) {
        RCLCPP_ERROR(logger_, "Failed to setup joint mapping");
        return false;
    }
    
    // Initialize CAN manager
    if (!initializeCanManager()) {
        RCLCPP_ERROR(logger_, "Failed to initialize CAN manager");
        return false;
    }
    
    RCLCPP_INFO(logger_, "Hardware interface initialized successfully");
    return true;
}

bool ActuatorInterface::configure() {
    RCLCPP_INFO(logger_, "Configuring hardware interface");
    
    // Initialize joint states and commands
    joint_positions_.resize(joint_names_.size(), 0.0);
    joint_velocities_.resize(joint_names_.size(), 0.0);
    joint_efforts_.resize(joint_names_.size(), 0.0);
    joint_position_commands_.resize(joint_names_.size(), 0.0);
    joint_velocity_commands_.resize(joint_names_.size(), 0.0);
    joint_effort_commands_.resize(joint_names_.size(), 0.0);
    
    RCLCPP_INFO(logger_, "Hardware interface configured with %zu joints", joint_names_.size());
    return true;
}

bool ActuatorInterface::activate() {
    RCLCPP_INFO(logger_, "Activating hardware interface");
    
    if (!can_manager_) {
        RCLCPP_ERROR(logger_, "CAN manager not initialized");
        return false;
    }
    
    // Start CAN manager
    if (!can_manager_->start()) {
        RCLCPP_ERROR(logger_, "Failed to start CAN manager");
        return false;
    }
    
    RCLCPP_INFO(logger_, "Hardware interface activated successfully");
    return true;
}

bool ActuatorInterface::deactivate() {
    RCLCPP_INFO(logger_, "Deactivating hardware interface");
    
    if (can_manager_) {
        can_manager_->close();
    }
    
    RCLCPP_INFO(logger_, "Hardware interface deactivated");
    return true;
}

void ActuatorInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!can_manager_) {
        return;
    }
    
    // Read from CAN devices
    can_manager_->read();
    
    // Update joint states from actuator data
    const auto& actuator_devices = can_manager_->getActuatorDevices();
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& joint_name = joint_names_[i];
        auto it = joint_to_actuator_map_.find(joint_name);
        
        if (it != joint_to_actuator_map_.end()) {
            const std::string& actuator_name = it->second;
            auto actuator_it = actuator_devices.find(actuator_name);
            
            if (actuator_it != actuator_devices.end()) {
                auto actuator = actuator_it->second;
                joint_positions_[i] = actuator->getPosition();
                joint_velocities_[i] = actuator->getVelocity();
                joint_efforts_[i] = actuator->getEffort();
            }
        }
    }
}

void ActuatorInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!can_manager_) {
        return;
    }
    
    // Update actuator commands from joint commands
    const auto& actuator_devices = can_manager_->getActuatorDevices();
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string& joint_name = joint_names_[i];
        auto it = joint_to_actuator_map_.find(joint_name);
        
        if (it != joint_to_actuator_map_.end()) {
            const std::string& actuator_name = it->second;
            auto actuator_it = actuator_devices.find(actuator_name);
            
            if (actuator_it != actuator_devices.end()) {
                auto actuator = actuator_it->second;
                
                // Set commands based on control mode
                // For velocity control mode
                actuator->setCommand(
                    joint_position_commands_[i],  // position
                    joint_velocity_commands_[i],  // velocity
                    0.0,                         // kp
                    0.0,                         // kd
                    joint_effort_commands_[i]    // effort
                );
            }
        }
    }
    
    // Write to CAN devices
    can_manager_->write();
}

std::vector<double*> ActuatorInterface::getJointPositionPtrs() {
    std::vector<double*> ptrs;
    for (auto& pos : joint_positions_) {
        ptrs.push_back(&pos);
    }
    return ptrs;
}

std::vector<double*> ActuatorInterface::getJointVelocityPtrs() {
    std::vector<double*> ptrs;
    for (auto& vel : joint_velocities_) {
        ptrs.push_back(&vel);
    }
    return ptrs;
}

std::vector<double*> ActuatorInterface::getJointEffortPtrs() {
    std::vector<double*> ptrs;
    for (auto& eff : joint_efforts_) {
        ptrs.push_back(&eff);
    }
    return ptrs;
}

std::vector<double*> ActuatorInterface::getJointPositionCommandPtrs() {
    std::vector<double*> ptrs;
    for (auto& cmd : joint_position_commands_) {
        ptrs.push_back(&cmd);
    }
    return ptrs;
}

std::vector<double*> ActuatorInterface::getJointVelocityCommandPtrs() {
    std::vector<double*> ptrs;
    for (auto& cmd : joint_velocity_commands_) {
        ptrs.push_back(&cmd);
    }
    return ptrs;
}

std::vector<double*> ActuatorInterface::getJointEffortCommandPtrs() {
    std::vector<double*> ptrs;
    for (auto& cmd : joint_effort_commands_) {
        ptrs.push_back(&cmd);
    }
    return ptrs;
}

bool ActuatorInterface::loadConfiguration() {
    try {
        // Get configuration file parameter (parameter should already be declared by ros2_control wrapper)
        config_file_ = node_->get_parameter("config_file").as_string();
        
        if (config_file_.empty()) {
            RCLCPP_ERROR(logger_, "No config file specified");
            return false;
        }
        
        RCLCPP_INFO(logger_, "Loading configuration from: %s", config_file_.c_str());
        
        // Configuration file is only used for CAN hardware setup
        // Joint names come from URDF via ros2_control framework
        RCLCPP_INFO(logger_, "Configuration file path: %s", config_file_.c_str());
        RCLCPP_INFO(logger_, "Joint names will be provided by ros2_control framework from URDF");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception loading configuration: %s", e.what());
        return false;
    }
}

bool ActuatorInterface::setupJointMapping() {
    // Joint mapping is now handled automatically using naming convention
    // joint1 -> motor1, joint2 -> motor2, etc.
    
    joint_to_actuator_map_.clear();
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        std::string joint_name = joint_names_[i];
        std::string actuator_name = "motor" + std::to_string(i + 1);
        
        joint_to_actuator_map_[joint_name] = actuator_name;
        
        RCLCPP_INFO(logger_, "Auto-mapped joint '%s' to actuator '%s'", 
                   joint_name.c_str(), actuator_name.c_str());
    }
    
    RCLCPP_INFO(logger_, "Joint mapping setup complete with %zu joints using naming convention", joint_names_.size());
    return true;
}

bool ActuatorInterface::initializeCanManager() {
    try {
        // Create CAN node
        can_node_ = std::make_shared<rclcpp::Node>("can_node");
        
        // Initialize CAN manager
        can_manager_ = std::make_shared<device::CanManager>(can_node_);
        
        // Load CAN configuration from the same YAML file
        if (!can_manager_->loadConfigFromFile(config_file_)) {
            RCLCPP_ERROR(logger_, "Failed to load CAN configuration from file");
            return false;
        }
        
        if (!can_manager_->init()) {
            RCLCPP_ERROR(logger_, "Failed to initialize CAN manager");
            return false;
        }
        
        RCLCPP_INFO(logger_, "CAN manager initialized successfully");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception initializing CAN manager: %s", e.what());
        return false;
    }
}

void ActuatorInterface::resizeJointArrays() {
    size_t joint_count = joint_names_.size();
    
    joint_positions_.resize(joint_count, 0.0);
    joint_velocities_.resize(joint_count, 0.0);
    joint_efforts_.resize(joint_count, 0.0);
    
    joint_position_commands_.resize(joint_count, 0.0);
    joint_velocity_commands_.resize(joint_count, 0.0);
    joint_effort_commands_.resize(joint_count, 0.0);
    
    RCLCPP_INFO(logger_, "Resized joint arrays for %zu joints", joint_count);
}

void ActuatorInterface::addJoint(const std::string& joint_name, const std::string& actuator_name) {
    // Add joint name to the list
    joint_names_.push_back(joint_name);
    
    // Map joint to actuator
    joint_to_actuator_map_[joint_name] = actuator_name;
    
    // Resize arrays to accommodate the new joint
    resizeJointArrays();
    
    RCLCPP_INFO(logger_, "Added joint '%s' mapped to actuator '%s'", 
               joint_name.c_str(), actuator_name.c_str());
}

} // namespace actuator_interface 