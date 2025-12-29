#include "hk_light_controller/hk_light_controller_node.hpp"
#include <chrono>
#include <cstring>
#include <sstream>

HkLightControllerNode::HkLightControllerNode()
  : Node("hk_light_controller_node"),
    interface_handle_(nullptr),
    interface_initialized_(false),
    using_device_handle_(false),
    light1_enabled_(false),
    light2_enabled_(false),
    light1_brightness_(0),
    light2_brightness_(0),
    current_trigger_source_("Unknown")
{
  // Declare parameters
  this->declare_parameter<std::string>("ip_address", "192.168.1.100");
  this->declare_parameter<double>("status_publish_rate", 1.0);  // Hz

  // Get parameters
  target_ip_address_ = this->get_parameter("ip_address").as_string();
  double status_rate = this->get_parameter("status_publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Initializing Hikvision light controller...");
  RCLCPP_INFO(this->get_logger(), "Target IP address: %s", target_ip_address_.c_str());

  // Create publishers
  light1_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("light1/status", 10);
  light2_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("light2/status", 10);
  light1_brightness_pub_ = this->create_publisher<std_msgs::msg::Int32>("light1/brightness", 10);
  light2_brightness_pub_ = this->create_publisher<std_msgs::msg::Int32>("light2/brightness", 10);
  trigger_source_pub_ = this->create_publisher<std_msgs::msg::String>("trigger_source", 10);
  connection_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("connection_status", 10);

  // Create subscribers
  light1_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "light1/control", 10,
    std::bind(&HkLightControllerNode::light1ControlCallback, this, std::placeholders::_1));
  light2_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "light2/control", 10,
    std::bind(&HkLightControllerNode::light2ControlCallback, this, std::placeholders::_1));
  light1_brightness_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "light1/set_brightness", 10,
    std::bind(&HkLightControllerNode::light1BrightnessCallback, this, std::placeholders::_1));
  light2_brightness_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "light2/set_brightness", 10,
    std::bind(&HkLightControllerNode::light2BrightnessCallback, this, std::placeholders::_1));
  trigger_source_sub_ = this->create_subscription<std_msgs::msg::String>(
    "set_trigger_source", 10,
    std::bind(&HkLightControllerNode::triggerSourceCallback, this, std::placeholders::_1));

  // Create services
  light1_service_ = this->create_service<std_srvs::srv::SetBool>(
    "light1/set_enabled",
    std::bind(&HkLightControllerNode::light1ServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  light2_service_ = this->create_service<std_srvs::srv::SetBool>(
    "light2/set_enabled",
    std::bind(&HkLightControllerNode::light2ServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  get_status_service_ = this->create_service<std_srvs::srv::Trigger>(
    "get_status",
    std::bind(&HkLightControllerNode::getStatusServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Initialize interface
  if (initializeInterface()) {
    RCLCPP_INFO(this->get_logger(), "Light controller initialized successfully");
    
    // Create status timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / status_rate));
    status_timer_ = this->create_wall_timer(
      timer_period, std::bind(&HkLightControllerNode::publishStatus, this));
    
    // Initial status update
    updateStatus();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize light controller");
    RCLCPP_ERROR(this->get_logger(), "Please check:");
    RCLCPP_ERROR(this->get_logger(), "  1. Light controller is connected and powered on");
    RCLCPP_ERROR(this->get_logger(), "  2. Network connection to %s is available", target_ip_address_.c_str());
    RCLCPP_ERROR(this->get_logger(), "  3. SDK environment variables are set (MVCAM_SDK_PATH, LD_LIBRARY_PATH)");
    RCLCPP_ERROR(this->get_logger(), "  4. Run SDK setup script: source SDK/MVS-4.6.1_x86_64_20251217/set_env_path.sh");
    // Don't call shutdown here - let the node continue but mark as not initialized
    // The main function will handle cleanup
  }
}

HkLightControllerNode::~HkLightControllerNode()
{
  cleanupInterface();
}

bool HkLightControllerNode::initializeInterface()
{
  int nRet = MV_OK;

  // Initialize SDK
  nRet = MV_CC_Initialize();
  if (MV_OK != nRet) {
    RCLCPP_ERROR(this->get_logger(), "Initialize SDK failed! nRet [0x%x]", nRet);
    return false;
  }

  // Try to enumerate light controller interfaces first
  MV_INTERFACE_INFO_LIST stInterfaceInfoList;
  memset(&stInterfaceInfoList, 0, sizeof(MV_INTERFACE_INFO_LIST));
  nRet = MV_CC_EnumInterfaces(MV_LC_INTERFACE, &stInterfaceInfoList);
  
  MV_INTERFACE_INFO* pTargetInterface = nullptr;
  bool found = false;
  
  if (MV_OK == nRet && stInterfaceInfoList.nInterfaceNum > 0) {
    RCLCPP_INFO(this->get_logger(), "Found %d light controller interface(s) via MV_LC_INTERFACE", 
                stInterfaceInfoList.nInterfaceNum);
    
    // Find interface by IP address
    for (unsigned int i = 0; i < stInterfaceInfoList.nInterfaceNum; i++) {
      MV_INTERFACE_INFO* pstInterfaceInfo = stInterfaceInfoList.pInterfaceInfos[i];
      if (NULL == pstInterfaceInfo) {
        continue;
      }

      std::string interface_id((char*)pstInterfaceInfo->chInterfaceID);
      std::string display_name((char*)pstInterfaceInfo->chDisplayName);
      
      RCLCPP_INFO(this->get_logger(), "Interface %d: ID=%s, DisplayName=%s", 
                  i, interface_id.c_str(), display_name.c_str());
      
      if (interface_id.find(target_ip_address_) != std::string::npos ||
          display_name.find(target_ip_address_) != std::string::npos) {
        pTargetInterface = pstInterfaceInfo;
        found = true;
        RCLCPP_INFO(this->get_logger(), "Found matching interface at index %d", i);
        break;
      }
    }
    
    if (!found && stInterfaceInfoList.nInterfaceNum > 0) {
      pTargetInterface = stInterfaceInfoList.pInterfaceInfos[0];
      found = true;
      RCLCPP_WARN(this->get_logger(), "Using first available interface");
    }
  } else {
    // If MV_LC_INTERFACE enumeration fails or returns 0, try enumerating GigE interfaces
    // Some light controllers appear as GigE interfaces instead of LC interfaces
    RCLCPP_WARN(this->get_logger(), 
                "No interfaces found via MV_LC_INTERFACE, trying GigE interface enumeration...");
    
    // First try GigE interfaces
    MV_INTERFACE_INFO_LIST stGigEInterfaceList;
    memset(&stGigEInterfaceList, 0, sizeof(MV_INTERFACE_INFO_LIST));
    nRet = MV_CC_EnumInterfaces(MV_GIGE_INTERFACE, &stGigEInterfaceList);
    
    if (MV_OK == nRet && stGigEInterfaceList.nInterfaceNum > 0) {
      RCLCPP_INFO(this->get_logger(), "Found %d GigE interface(s)", stGigEInterfaceList.nInterfaceNum);
      
      // Find interface by IP address
      for (unsigned int i = 0; i < stGigEInterfaceList.nInterfaceNum; i++) {
        MV_INTERFACE_INFO* pstInterfaceInfo = stGigEInterfaceList.pInterfaceInfos[i];
        if (NULL == pstInterfaceInfo) {
          continue;
        }

        std::string interface_id((char*)pstInterfaceInfo->chInterfaceID);
        std::string display_name((char*)pstInterfaceInfo->chDisplayName);
        
        RCLCPP_INFO(this->get_logger(), "GigE Interface %d: ID=%s, DisplayName=%s", 
                    i, interface_id.c_str(), display_name.c_str());
        
        if (interface_id.find(target_ip_address_) != std::string::npos ||
            display_name.find(target_ip_address_) != std::string::npos) {
          pTargetInterface = pstInterfaceInfo;
          found = true;
          RCLCPP_INFO(this->get_logger(), "Found matching GigE interface at index %d", i);
          break;
        }
      }
    }
    
    // If GigE interfaces also fail, try enumerating GigE devices as fallback
    if (!found) {
      RCLCPP_WARN(this->get_logger(), 
                  "No GigE interfaces found, trying GigE device enumeration as fallback...");
      
      MV_CC_DEVICE_INFO_LIST stDeviceList;
      memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
      nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
    
      if (MV_OK == nRet && stDeviceList.nDeviceNum > 0 && !found) {
        RCLCPP_INFO(this->get_logger(), "Found %d GigE device(s)", stDeviceList.nDeviceNum);
      
        // Find device by IP address
        // SDK stores IP in a specific format: when displayed as bytes, it shows as reversed
        // Device IP "100.1.168.192" in display means actual IP is "192.168.1.100"
        // SDK format: bytes are (ip4, ip3, ip2, ip1) where ip1 is lowest byte
        // So we need to compare by converting both to the same format
        int targetIpParts[4];
        if (sscanf(target_ip_address_.c_str(), "%d.%d.%d.%d", 
                   &targetIpParts[0], &targetIpParts[1], &targetIpParts[2], &targetIpParts[3]) != 4) {
          RCLCPP_ERROR(this->get_logger(), "Invalid IP address format: %s", target_ip_address_.c_str());
          MV_CC_Finalize();
          return false;
        }
        
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
          MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
          if (NULL == pDeviceInfo || pDeviceInfo->nTLayerType != MV_GIGE_DEVICE) {
            continue;
          }
          
          unsigned int deviceIp = pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp;
          // Extract bytes from SDK format
          int deviceIpParts[4];
          deviceIpParts[0] = (deviceIp & 0xff);
          deviceIpParts[1] = ((deviceIp >> 8) & 0xff);
          deviceIpParts[2] = ((deviceIp >> 16) & 0xff);
          deviceIpParts[3] = ((deviceIp >> 24) & 0xff);
          
          // Display IP (reversed for human reading)
          char ip_str[16];
          snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
                   deviceIpParts[0], deviceIpParts[1], deviceIpParts[2], deviceIpParts[3]);
          
          // Actual IP (correct order)
          char actual_ip_str[16];
          snprintf(actual_ip_str, sizeof(actual_ip_str), "%d.%d.%d.%d",
                   deviceIpParts[3], deviceIpParts[2], deviceIpParts[1], deviceIpParts[0]);
          
          RCLCPP_INFO(this->get_logger(), "Device %d: IP=%s (actual: %s), Model=%s, Serial=%s", 
                      i, ip_str, actual_ip_str,
                      pDeviceInfo->SpecialInfo.stGigEInfo.chModelName,
                      pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
          
          // Compare actual IP addresses
          if (deviceIpParts[0] == targetIpParts[3] &&
              deviceIpParts[1] == targetIpParts[2] &&
              deviceIpParts[2] == targetIpParts[1] &&
              deviceIpParts[3] == targetIpParts[0]) {
            RCLCPP_INFO(this->get_logger(), "Found matching device at IP %s (actual: %s)", 
                        ip_str, actual_ip_str);
            
            // Try multiple interface ID formats
            std::vector<std::string> interfaceIdFormats = {
              std::string("GigE:") + actual_ip_str,      // GigE:192.168.1.100
              std::string("GigE:") + ip_str,             // GigE:100.1.168.192
              actual_ip_str,                              // 192.168.1.100
              ip_str,                                     // 100.1.168.192
              std::string("GigEVision:") + actual_ip_str // GigEVision:192.168.1.100
            };
            
            bool interfaceCreated = false;
            for (const auto& interfaceId : interfaceIdFormats) {
              RCLCPP_DEBUG(this->get_logger(), "Trying interface ID: %s", interfaceId.c_str());
              nRet = MV_CC_CreateInterfaceByID(&interface_handle_, interfaceId.c_str());
              if (MV_OK == nRet) {
                found = true;
                interfaceCreated = true;
                RCLCPP_INFO(this->get_logger(), "Successfully created interface using ID: %s", 
                            interfaceId.c_str());
                break;
              }
            }
            
            if (!interfaceCreated) {
              RCLCPP_WARN(this->get_logger(), 
                          "Failed to create interface by ID. "
                          "Trying to enumerate GigE interfaces...");
              
              // Try enumerating GigE interfaces to find matching one
              MV_INTERFACE_INFO_LIST stGigEInterfaceList = {0};
              nRet = MV_CC_EnumInterfaces(MV_GIGE_INTERFACE, &stGigEInterfaceList);
              if (MV_OK == nRet && stGigEInterfaceList.nInterfaceNum > 0) {
                RCLCPP_INFO(this->get_logger(), "Found %d GigE interfaces", stGigEInterfaceList.nInterfaceNum);
                
                // Log all interfaces for debugging
                for (unsigned int j = 0; j < stGigEInterfaceList.nInterfaceNum; j++) {
                  MV_INTERFACE_INFO* pInterfaceInfo = stGigEInterfaceList.pInterfaceInfos[j];
                  if (pInterfaceInfo == NULL) continue;
                  std::string interfaceId = reinterpret_cast<const char*>(pInterfaceInfo->chInterfaceID);
                  std::string displayName = reinterpret_cast<const char*>(pInterfaceInfo->chDisplayName);
                  RCLCPP_DEBUG(this->get_logger(), 
                               "GigE Interface %d: ID=%s, DisplayName=%s", 
                               j, interfaceId.c_str(), displayName.c_str());
                }
                
                // Search for interface matching the device IP
                for (unsigned int j = 0; j < stGigEInterfaceList.nInterfaceNum; j++) {
                  MV_INTERFACE_INFO* pInterfaceInfo = stGigEInterfaceList.pInterfaceInfos[j];
                  if (pInterfaceInfo == NULL) continue;
                  
                  // Check if interface ID or display name contains the IP
                  std::string interfaceId = reinterpret_cast<const char*>(pInterfaceInfo->chInterfaceID);
                  std::string displayName = reinterpret_cast<const char*>(pInterfaceInfo->chDisplayName);
                  
                  RCLCPP_DEBUG(this->get_logger(), 
                               "Checking interface %d: ID=%s, DisplayName=%s against IPs: %s, %s", 
                               j, interfaceId.c_str(), displayName.c_str(), actual_ip_str, ip_str);
                  
                  if (interfaceId.find(actual_ip_str) != std::string::npos ||
                      interfaceId.find(ip_str) != std::string::npos ||
                      displayName.find(actual_ip_str) != std::string::npos ||
                      displayName.find(ip_str) != std::string::npos) {
                    RCLCPP_INFO(this->get_logger(), 
                                "Found matching GigE interface: %s (%s)", 
                                interfaceId.c_str(), displayName.c_str());
                    
                    nRet = MV_CC_CreateInterface(&interface_handle_, pInterfaceInfo);
                    if (MV_OK == nRet) {
                      // Open interface first
                      nRet = MV_CC_OpenInterface(interface_handle_, NULL);
                      if (MV_OK == nRet) {
                        // Try to enumerate devices through interface to verify it's the right one
                        MV_CC_DEVICE_INFO_LIST stDeviceListByInterface = {0};
                        int enumRet = MV_CC_EnumDevicesByInterface(interface_handle_, &stDeviceListByInterface);
                        if (enumRet == MV_OK && stDeviceListByInterface.nDeviceNum > 0) {
                          // Check if any device matches our IP
                          bool deviceFound = false;
                          for (unsigned int k = 0; k < stDeviceListByInterface.nDeviceNum; k++) {
                            MV_CC_DEVICE_INFO* pDevInfo = stDeviceListByInterface.pDeviceInfo[k];
                            if (pDevInfo && pDevInfo->nTLayerType == MV_GIGE_DEVICE) {
                              unsigned int devIp = pDevInfo->SpecialInfo.stGigEInfo.nCurrentIp;
                              int devIpParts[4];
                              devIpParts[0] = (devIp & 0xff);
                              devIpParts[1] = ((devIp >> 8) & 0xff);
                              devIpParts[2] = ((devIp >> 16) & 0xff);
                              devIpParts[3] = ((devIp >> 24) & 0xff);
                              
                              if (devIpParts[0] == targetIpParts[3] &&
                                  devIpParts[1] == targetIpParts[2] &&
                                  devIpParts[2] == targetIpParts[1] &&
                                  devIpParts[3] == targetIpParts[0]) {
                                deviceFound = true;
                                break;
                              }
                            }
                          }
                          
                          if (deviceFound) {
                            found = true;
                            interfaceCreated = true;
                            RCLCPP_INFO(this->get_logger(), 
                                        "Successfully created and opened interface from GigE interface list");
                            break;
                          } else {
                            // Device not found through this interface, close and try next
                            MV_CC_CloseInterface(interface_handle_);
                            MV_CC_DestroyInterface(interface_handle_);
                            interface_handle_ = nullptr;
                          }
                        } else {
                          // No devices found, but interface might still work for light controller
                          found = true;
                          interfaceCreated = true;
                          RCLCPP_INFO(this->get_logger(), 
                                      "Successfully created and opened interface (no devices enumerated, but interface opened)");
                          break;
                        }
                      } else {
                        RCLCPP_WARN(this->get_logger(), 
                                    "Failed to open interface! nRet [0x%x]", nRet);
                        MV_CC_DestroyInterface(interface_handle_);
                        interface_handle_ = nullptr;
                      }
                    } else {
                      RCLCPP_WARN(this->get_logger(), 
                                  "Failed to create interface from GigE interface list! nRet [0x%x]", nRet);
                    }
                  }
                }
              } else {
                RCLCPP_WARN(this->get_logger(), 
                            "Failed to enumerate GigE interfaces or no interfaces found. nRet [0x%x], num=%d", 
                            nRet, stGigEInterfaceList.nInterfaceNum);
              }
              
              // Last resort: try using device handle (may not work for all operations)
              if (!interfaceCreated) {
                RCLCPP_WARN(this->get_logger(), 
                            "Failed to create interface from GigE interface list. "
                            "Trying to use device handle as fallback...");
                
                void* deviceHandle = nullptr;
                nRet = MV_CC_CreateHandle(&deviceHandle, pDeviceInfo);
                if (MV_OK == nRet) {
                  nRet = MV_CC_OpenDevice(deviceHandle, MV_ACCESS_Exclusive, 0);
                  if (MV_OK == nRet) {
                    // Try to use device handle as interface handle
                    // This is a workaround - test if device handle can access interface parameters
                    RCLCPP_WARN(this->get_logger(), 
                                "Using device handle as interface handle (experimental). "
                                "This may not work for all operations.");
                    interface_handle_ = deviceHandle;
                    found = true;
                    interfaceCreated = true;
                    using_device_handle_ = true;  // Mark that we're using device handle
                    RCLCPP_INFO(this->get_logger(), "Using device handle for light controller operations");
                  } else {
                    MV_CC_DestroyHandle(deviceHandle);
                    RCLCPP_ERROR(this->get_logger(), "Failed to open device handle! nRet [0x%x]", nRet);
                  }
                } else {
                  RCLCPP_ERROR(this->get_logger(), "Failed to create device handle! nRet [0x%x]", nRet);
                }
                
                if (!interfaceCreated) {
                  RCLCPP_ERROR(this->get_logger(), 
                              "All methods failed. This device model may not support light controller interface API.");
                  RCLCPP_ERROR(this->get_logger(), 
                              "Please check SDK documentation for device model: %s", 
                              pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
                }
              }
            }
            break;
          }
        }
      }
    }
  }

  if (!found) {
    RCLCPP_ERROR(this->get_logger(), "No light controller found at IP %s", target_ip_address_.c_str());
    RCLCPP_ERROR(this->get_logger(), "Please check:");
    RCLCPP_ERROR(this->get_logger(), "  1. Device is connected and powered on");
    RCLCPP_ERROR(this->get_logger(), "  2. IP address is correct: %s", target_ip_address_.c_str());
    RCLCPP_ERROR(this->get_logger(), "  3. Network connection is available");
    MV_CC_Finalize();
    return false;
  }

  // Create interface handle if we found an interface (not created by ID or device handle)
  if (pTargetInterface != nullptr && !using_device_handle_) {
    nRet = MV_CC_CreateInterface(&interface_handle_, pTargetInterface);
    if (MV_OK != nRet) {
      RCLCPP_ERROR(this->get_logger(), "Create Interface failed! nRet [0x%x]", nRet);
      MV_CC_Finalize();
      return false;
    }
  } else if (interface_handle_ == nullptr && !using_device_handle_) {
    // Interface should have been created by ID, but check if it failed
    RCLCPP_ERROR(this->get_logger(), "Interface handle is null after creation attempt");
    MV_CC_Finalize();
    return false;
  }

  // Open interface (or device if using device handle)
  if (using_device_handle_) {
    // Device is already opened, just mark as initialized
    RCLCPP_INFO(this->get_logger(), "Device handle already opened, skipping interface open");
    RCLCPP_WARN(this->get_logger(), 
                "WARNING: Using device handle. Light controller parameters may not work correctly. "
                "Please ensure the device supports device handle API for light control.");
  } else {
    // Check if interface is already opened (might have been opened during creation)
    // Try to get a parameter to check if it's already open
    bool testValue = false;
    int testRet = MV_CC_GetBoolValue(interface_handle_, "LineStatus", &testValue);
    if (testRet != MV_OK) {
      // Interface not open yet, open it
      nRet = MV_CC_OpenInterface(interface_handle_, NULL);
      if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Open Interface failed! nRet [0x%x]", nRet);
        MV_CC_DestroyInterface(interface_handle_);
        interface_handle_ = nullptr;
        MV_CC_Finalize();
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Interface opened successfully");
    } else {
      RCLCPP_INFO(this->get_logger(), "Interface already opened");
    }
  }

  interface_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "Interface opened successfully");

  // Read initial state
  updateStatus();

  return true;
}

void HkLightControllerNode::cleanupInterface()
{
  if (interface_handle_ != nullptr) {
    if (interface_initialized_) {
      if (using_device_handle_) {
        MV_CC_CloseDevice(interface_handle_);
        MV_CC_DestroyHandle(interface_handle_);
      } else {
        MV_CC_CloseInterface(interface_handle_);
        MV_CC_DestroyInterface(interface_handle_);
      }
    }
    interface_handle_ = nullptr;
  }
  MV_CC_Finalize();
  RCLCPP_INFO(this->get_logger(), "Interface cleaned up");
}

bool HkLightControllerNode::setLightState(int light_index, bool enabled)
{
  std::lock_guard<std::mutex> lock(control_mutex_);

  if (!interface_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Interface not initialized");
    return false;
  }

  if (light_index != 1 && light_index != 2) {
    RCLCPP_ERROR(this->get_logger(), "Invalid light index: %d (must be 1 or 2)", light_index);
    return false;
  }

  // Based on official MVS software interface:
  // - Light Controller Selector: Selects channel (Light Controller 1 or 2)
  // - Light Controller Source: Controls on/off (On, Off, Timer 1, Timer 2, etc.)
  // - Light Brightness: Controls brightness
  
  // Step 1: Set Light Controller Selector to select the channel
  int current_selector = 0;
  if (getEnumValue("LightControllerSelector", current_selector)) {
    int target_selector = light_index;  // 1 or 2
    if (current_selector != target_selector) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Setting LightControllerSelector from %d to %d for light %d", 
                   current_selector, target_selector, light_index);
      setEnumValue("LightControllerSelector", target_selector);
    } else {
      RCLCPP_DEBUG(this->get_logger(), 
                   "LightControllerSelector already at value %d for light %d", 
                   target_selector, light_index);
    }
  } else {
    // Fallback: Try alternative parameter names
    RCLCPP_DEBUG(this->get_logger(), "LightControllerSelector not found, trying alternatives");
    if (getEnumValue("LightSelector", current_selector)) {
      int target_selector = light_index;
      if (current_selector != target_selector) {
        setEnumValue("LightSelector", target_selector);
      }
    }
  }
  
  // Step 2: Set Light Controller Source to control on/off
  // Values: 0=Off, 1=On, or use symbolic values
  bool success = false;
  
  // Try setting LightControllerSource using enum value
  // First, try to get the enum to find the correct value for "On" or "Off"
  MVCC_ENUMVALUE stSourceEnum;
  memset(&stSourceEnum, 0, sizeof(MVCC_ENUMVALUE));
  int nRet = MV_CC_GetEnumValue(interface_handle_, "LightControllerSource", &stSourceEnum);
  if (MV_OK == nRet) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "LightControllerSource current value: %u, supported count: %u", 
                 stSourceEnum.nCurValue, stSourceEnum.nSupportedNum);
    
    // Try to find "On" or "Off" value
    // Usually: 0=Off, 1=On (but need to verify)
    unsigned int target_value = enabled ? 1 : 0;
    
    // Check if target value is in supported list
    bool value_found = false;
    for (unsigned int i = 0; i < stSourceEnum.nSupportedNum; i++) {
      if (stSourceEnum.nSupportValue[i] == target_value) {
        value_found = true;
        break;
      }
    }
    
    if (value_found) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Setting LightControllerSource to %u (%s)", 
                   target_value, enabled ? "On" : "Off");
      nRet = MV_CC_SetEnumValue(interface_handle_, "LightControllerSource", target_value);
      if (MV_OK == nRet) {
        success = true;
        RCLCPP_INFO(this->get_logger(), 
                   "Successfully set LightControllerSource to %u for light %d", 
                   target_value, light_index);
      } else {
        RCLCPP_WARN(this->get_logger(), 
                    "Failed to set LightControllerSource to %u, nRet [0x%x]", 
                    target_value, nRet);
      }
    } else {
      // Try symbolic value
      RCLCPP_DEBUG(this->get_logger(), "Value not found, trying symbolic value");
      const char* symbolic_value = enabled ? "On" : "Off";
      nRet = MV_CC_SetEnumValueByString(interface_handle_, "LightControllerSource", symbolic_value);
      if (MV_OK == nRet) {
        success = true;
        RCLCPP_INFO(this->get_logger(), 
                   "Successfully set LightControllerSource to '%s' for light %d", 
                   symbolic_value, light_index);
      } else {
        RCLCPP_WARN(this->get_logger(), 
                    "Failed to set LightControllerSource to '%s', nRet [0x%x]", 
                    symbolic_value, nRet);
      }
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), 
                 "Cannot read LightControllerSource, trying alternative parameter names");
    
    // Try alternative parameter names
    const char* altParams[] = {
      "LightSource", "ControllerSource", "Source"
    };
    
    for (size_t i = 0; i < sizeof(altParams)/sizeof(altParams[0]) && !success; i++) {
      nRet = MV_CC_GetEnumValue(interface_handle_, altParams[i], &stSourceEnum);
      if (MV_OK == nRet) {
        unsigned int target_value = enabled ? 1 : 0;
        nRet = MV_CC_SetEnumValue(interface_handle_, altParams[i], target_value);
        if (MV_OK == nRet) {
          success = true;
          RCLCPP_INFO(this->get_logger(), 
                     "Successfully set %s to %u for light %d", 
                     altParams[i], target_value, light_index);
        }
      }
    }
  }
  
  if (success) {
    if (light_index == 1) {
      light1_enabled_ = enabled;
    } else {
      light2_enabled_ = enabled;
    }
    RCLCPP_INFO(this->get_logger(), "Light %d set to %s", light_index, enabled ? "ON" : "OFF");
  } else {
    RCLCPP_ERROR(this->get_logger(), 
                 "Failed to set light %d state. Tried LineStatus, StrobeEnable, OutputEnable", 
                 light_index);
  }

  return success;
}

bool HkLightControllerNode::setLightBrightness(int light_index, int64_t brightness)
{
  std::lock_guard<std::mutex> lock(control_mutex_);

  if (!interface_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Interface not initialized");
    return false;
  }

  if (light_index != 1 && light_index != 2) {
    RCLCPP_ERROR(this->get_logger(), "Invalid light index: %d (must be 1 or 2)", light_index);
    return false;
  }

  // Step 1: Set Light Controller Selector to select the channel
  int current_selector = 0;
  if (getEnumValue("LightControllerSelector", current_selector)) {
    int target_selector = light_index;  // 1 or 2
    if (current_selector != target_selector) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Setting LightControllerSelector from %d to %d for light %d", 
                   current_selector, target_selector, light_index);
      setEnumValue("LightControllerSelector", target_selector);
    }
  } else {
    // Fallback: Try alternative parameter names
    if (getEnumValue("LightSelector", current_selector)) {
      int target_selector = light_index;
      if (current_selector != target_selector) {
        setEnumValue("LightSelector", target_selector);
      }
    }
  }

  // Step 2: Set Light Brightness
  // According to official MVS software, parameter name is "LightBrightness"
  bool success = false;
  
  // Try LightBrightness (as shown in official MVS software)
  success = setIntValue("LightBrightness", brightness);
  
  if (!success) {
    // Try alternative parameter names
    RCLCPP_DEBUG(this->get_logger(), "LightBrightness failed, trying Brightness");
    success = setIntValue("Brightness", brightness);
  }
  
  if (!success) {
    RCLCPP_DEBUG(this->get_logger(), "Brightness failed, trying Intensity");
    success = setIntValue("Intensity", brightness);
  }
  
  if (!success) {
    // Try with light-specific parameter names
    char paramName[64];
    snprintf(paramName, sizeof(paramName), "Light%dBrightness", light_index);
    RCLCPP_DEBUG(this->get_logger(), "Trying light-specific parameter: %s", paramName);
    success = setIntValue(paramName, brightness);
  }
  
  if (success) {
    if (light_index == 1) {
      light1_brightness_ = brightness;
    } else {
      light2_brightness_ = brightness;
    }
    RCLCPP_INFO(this->get_logger(), "Light %d brightness set to %ld", light_index, brightness);
  } else {
    RCLCPP_WARN(this->get_logger(), 
                "Failed to set brightness for light %d. "
                "Tried: LineSource, Brightness, Intensity, StrobeLineDuration. "
                "Your controller might use a different parameter name.", 
                light_index);
  }

  return success;
}

bool HkLightControllerNode::setTriggerSource(const std::string& trigger_source)
{
  std::lock_guard<std::mutex> lock(control_mutex_);

  if (!interface_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Interface not initialized");
    return false;
  }

  // Map trigger source string to enum value
  int trigger_value = 0;
  if (trigger_source == "Software" || trigger_source == "software") {
    trigger_value = 0;  // Software trigger
  } else if (trigger_source == "Line0" || trigger_source == "line0") {
    trigger_value = 1;
  } else if (trigger_source == "Line1" || trigger_source == "line1") {
    trigger_value = 2;
  } else if (trigger_source == "Line2" || trigger_source == "line2") {
    trigger_value = 3;
  } else if (trigger_source == "Line3" || trigger_source == "line3") {
    trigger_value = 4;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown trigger source: %s, using Software", 
                trigger_source.c_str());
    trigger_value = 0;
  }

  bool success = setEnumValue("TimerTriggerSource", trigger_value);
  
  if (success) {
    current_trigger_source_ = trigger_source;
    RCLCPP_INFO(this->get_logger(), "Trigger source set to: %s", trigger_source.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set trigger source");
  }

  return success;
}

void HkLightControllerNode::updateStatus()
{
  if (!interface_initialized_) {
    return;
  }

  std::lock_guard<std::mutex> lock(control_mutex_);

  // Read light 1 status
  // Step 1: Set Light Controller Selector to channel 1
  int current_selector = 0;
  if (getEnumValue("LightControllerSelector", current_selector)) {
    if (current_selector != 1) {
      setEnumValue("LightControllerSelector", 1);
    }
  } else {
    // Fallback: Try alternative parameter names
    if (getEnumValue("LightSelector", current_selector)) {
      if (current_selector != 1) {
        setEnumValue("LightSelector", 1);
      }
    }
  }

  // Step 2: Read Light Controller Source for light 1
  bool light1_status = false;
  MVCC_ENUMVALUE stSourceEnum1;
  memset(&stSourceEnum1, 0, sizeof(MVCC_ENUMVALUE));
  int nRet = MV_CC_GetEnumValue(interface_handle_, "LightControllerSource", &stSourceEnum1);
  if (MV_OK == nRet) {
    // Get symbolic value for current enum value
    std::string symbolic_value;
    if (getEnumEntrySymbolic("LightControllerSource", static_cast<int>(stSourceEnum1.nCurValue), symbolic_value)) {
      // Check if source is "On" (case-insensitive)
      light1_status = (symbolic_value == "On" || symbolic_value == "ON" || symbolic_value == "on");
    } else {
      // Fallback: Check numeric value (usually 1 = On, 0 = Off, but may vary)
      // Check all supported values to find which one is "On"
      for (unsigned int i = 0; i < stSourceEnum1.nSupportedNum; i++) {
        std::string test_symbolic;
        if (getEnumEntrySymbolic("LightControllerSource", static_cast<int>(stSourceEnum1.nSupportValue[i]), test_symbolic)) {
          if (test_symbolic == "On" || test_symbolic == "ON" || test_symbolic == "on") {
            light1_status = (stSourceEnum1.nCurValue == stSourceEnum1.nSupportValue[i]);
            break;
          }
        }
      }
      // If still not found, assume 1 = On, 0 = Off
      if (!light1_status && stSourceEnum1.nCurValue == 1) {
        light1_status = true;
      }
    }
    light1_enabled_ = light1_status;
  } else {
    // Fallback: Try old parameter names
    if (getBoolValue("LineStatus", light1_status)) {
      light1_enabled_ = light1_status;
    } else if (getBoolValue("StrobeEnable", light1_status)) {
      light1_enabled_ = light1_status;
    } else if (getBoolValue("OutputEnable", light1_status)) {
      light1_enabled_ = light1_status;
    }
  }

  // Read light 2 status
  // Step 1: Set Light Controller Selector to channel 2
  if (getEnumValue("LightControllerSelector", current_selector)) {
    if (current_selector != 2) {
      setEnumValue("LightControllerSelector", 2);
    }
  } else {
    // Fallback: Try alternative parameter names
    if (getEnumValue("LightSelector", current_selector)) {
      if (current_selector != 2) {
        setEnumValue("LightSelector", 2);
      }
    }
  }

  // Step 2: Read Light Controller Source for light 2
  bool light2_status = false;
  MVCC_ENUMVALUE stSourceEnum2;
  memset(&stSourceEnum2, 0, sizeof(MVCC_ENUMVALUE));
  nRet = MV_CC_GetEnumValue(interface_handle_, "LightControllerSource", &stSourceEnum2);
  if (MV_OK == nRet) {
    // Get symbolic value for current enum value
    std::string symbolic_value;
    if (getEnumEntrySymbolic("LightControllerSource", static_cast<int>(stSourceEnum2.nCurValue), symbolic_value)) {
      // Check if source is "On" (case-insensitive)
      light2_status = (symbolic_value == "On" || symbolic_value == "ON" || symbolic_value == "on");
    } else {
      // Fallback: Check numeric value
      // Check all supported values to find which one is "On"
      for (unsigned int i = 0; i < stSourceEnum2.nSupportedNum; i++) {
        std::string test_symbolic;
        if (getEnumEntrySymbolic("LightControllerSource", static_cast<int>(stSourceEnum2.nSupportValue[i]), test_symbolic)) {
          if (test_symbolic == "On" || test_symbolic == "ON" || test_symbolic == "on") {
            light2_status = (stSourceEnum2.nCurValue == stSourceEnum2.nSupportValue[i]);
            break;
          }
        }
      }
      // If still not found, assume 1 = On, 0 = Off
      if (!light2_status && stSourceEnum2.nCurValue == 1) {
        light2_status = true;
      }
    }
    light2_enabled_ = light2_status;
  } else {
    // Fallback: Try old parameter names
    if (getBoolValue("LineStatus", light2_status)) {
      light2_enabled_ = light2_status;
    } else if (getBoolValue("StrobeEnable", light2_status)) {
      light2_enabled_ = light2_status;
    } else if (getBoolValue("OutputEnable", light2_status)) {
      light2_enabled_ = light2_status;
    }
  }

  // Read brightness values
  // Light 1 brightness
  if (getEnumValue("LightControllerSelector", current_selector)) {
    if (current_selector != 1) {
      setEnumValue("LightControllerSelector", 1);
    }
  }
  int64_t brightness1 = 0;
  if (getIntValue("LightBrightness", brightness1)) {
    light1_brightness_ = brightness1;
  } else if (getIntValue("Brightness", brightness1)) {
    light1_brightness_ = brightness1;
  } else if (getIntValue("Intensity", brightness1)) {
    light1_brightness_ = brightness1;
  }

  // Light 2 brightness
  if (getEnumValue("LightControllerSelector", current_selector)) {
    if (current_selector != 2) {
      setEnumValue("LightControllerSelector", 2);
    }
  }
  int64_t brightness2 = 0;
  if (getIntValue("LightBrightness", brightness2)) {
    light2_brightness_ = brightness2;
  } else if (getIntValue("Brightness", brightness2)) {
    light2_brightness_ = brightness2;
  } else if (getIntValue("Intensity", brightness2)) {
    light2_brightness_ = brightness2;
  }

  // Read trigger source
  int trigger_value = 0;
  if (getEnumValue("TimerTriggerSource", trigger_value)) {
    std::string trigger_symbolic;
    if (getEnumEntrySymbolic("TimerTriggerSource", trigger_value, trigger_symbolic)) {
      current_trigger_source_ = trigger_symbolic;
    } else {
      // Fallback to numeric value or common names
      switch (trigger_value) {
        case 0:
          current_trigger_source_ = "Software";
          break;
        case 1:
          current_trigger_source_ = "Line0";
          break;
        case 2:
          current_trigger_source_ = "Line1";
          break;
        case 3:
          current_trigger_source_ = "Line2";
          break;
        case 4:
          current_trigger_source_ = "Line3";
          break;
        default:
          current_trigger_source_ = std::to_string(trigger_value);
          break;
      }
    }
  }
}

void HkLightControllerNode::publishStatus()
{
  if (!interface_initialized_) {
    std_msgs::msg::Bool conn_msg;
    conn_msg.data = false;
    connection_status_pub_->publish(conn_msg);
    return;
  }

  // Update status from hardware
  updateStatus();

  // Publish connection status
  std_msgs::msg::Bool conn_msg;
  conn_msg.data = true;
  connection_status_pub_->publish(conn_msg);

  // Publish light 1 status
  std_msgs::msg::Bool light1_msg;
  light1_msg.data = light1_enabled_;
  light1_status_pub_->publish(light1_msg);

  // Publish light 2 status
  std_msgs::msg::Bool light2_msg;
  light2_msg.data = light2_enabled_;
  light2_status_pub_->publish(light2_msg);

  // Publish brightness
  std_msgs::msg::Int32 brightness1_msg;
  brightness1_msg.data = static_cast<int32_t>(light1_brightness_);
  light1_brightness_pub_->publish(brightness1_msg);

  std_msgs::msg::Int32 brightness2_msg;
  brightness2_msg.data = static_cast<int32_t>(light2_brightness_);
  light2_brightness_pub_->publish(brightness2_msg);

  // Publish trigger source
  std_msgs::msg::String trigger_msg;
  trigger_msg.data = current_trigger_source_;
  trigger_source_pub_->publish(trigger_msg);
}

void HkLightControllerNode::light1ControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!interface_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Light controller not initialized, ignoring light1 control command");
    return;
  }
  setLightState(1, msg->data);
}

void HkLightControllerNode::light2ControlCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!interface_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Light controller not initialized, ignoring light2 control command");
    return;
  }
  setLightState(2, msg->data);
}

void HkLightControllerNode::light1BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!interface_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Light controller not initialized, ignoring light1 brightness command");
    return;
  }
  setLightBrightness(1, static_cast<int64_t>(msg->data));
}

void HkLightControllerNode::light2BrightnessCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!interface_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Light controller not initialized, ignoring light2 brightness command");
    return;
  }
  setLightBrightness(2, static_cast<int64_t>(msg->data));
}

void HkLightControllerNode::triggerSourceCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!interface_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Light controller not initialized, ignoring trigger source command");
    return;
  }
  setTriggerSource(msg->data);
}

void HkLightControllerNode::light1ServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!interface_initialized_) {
    response->success = false;
    response->message = "Light controller not initialized";
    return;
  }
  response->success = setLightState(1, request->data);
  if (response->success) {
    response->message = "Light 1 " + std::string(request->data ? "enabled" : "disabled");
  } else {
    response->message = "Failed to set light 1 state";
  }
}

void HkLightControllerNode::light2ServiceCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!interface_initialized_) {
    response->success = false;
    response->message = "Light controller not initialized";
    return;
  }
  response->success = setLightState(2, request->data);
  if (response->success) {
    response->message = "Light 2 " + std::string(request->data ? "enabled" : "disabled");
  } else {
    response->message = "Failed to set light 2 state";
  }
}

void HkLightControllerNode::getStatusServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused
  updateStatus();
  response->success = true;
  std::ostringstream ss;
  ss << "Light 1: " << (light1_enabled_ ? "ON" : "OFF") 
     << " (brightness: " << light1_brightness_ << "), "
     << "Light 2: " << (light2_enabled_ ? "ON" : "OFF")
     << " (brightness: " << light2_brightness_ << "), "
     << "Trigger: " << current_trigger_source_;
  response->message = ss.str();
}

// SDK helper methods
bool HkLightControllerNode::setBoolValue(const std::string& key, bool value)
{
  // First check if parameter exists and is readable
  bool testValue = false;
  int nRet = MV_CC_GetBoolValue(interface_handle_, key.c_str(), &testValue);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter %s does not exist or is not readable! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  
  // Check if parameter is writable
  MV_XML_AccessMode enAccessMode = AM_NI;
  nRet = MV_XML_GetNodeAccessMode(interface_handle_, key.c_str(), &enAccessMode);
  if (MV_OK == nRet) {
    if (enAccessMode != AM_RW && enAccessMode != AM_WO) {
      RCLCPP_DEBUG(this->get_logger(), "Parameter %s is not writable! AccessMode=%d", key.c_str(), enAccessMode);
      return false;
    }
  }
  
  // Try to set the value
  nRet = MV_CC_SetBoolValue(interface_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Set %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  return true;
}

bool HkLightControllerNode::getBoolValue(const std::string& key, bool& value)
{
  bool bValue = false;
  int nRet = MV_CC_GetBoolValue(interface_handle_, key.c_str(), &bValue);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Get %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  value = bValue;
  return true;
}

bool HkLightControllerNode::setIntValue(const std::string& key, int64_t value)
{
  int nRet = MV_CC_SetIntValueEx(interface_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Set %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  return true;
}

bool HkLightControllerNode::getIntValue(const std::string& key, int64_t& value)
{
  MVCC_INTVALUE_EX stIntValue;
  memset(&stIntValue, 0, sizeof(MVCC_INTVALUE_EX));
  int nRet = MV_CC_GetIntValueEx(interface_handle_, key.c_str(), &stIntValue);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Get %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  value = stIntValue.nCurValue;
  return true;
}

bool HkLightControllerNode::setEnumValue(const std::string& key, int value)
{
  int nRet = MV_CC_SetEnumValue(interface_handle_, key.c_str(), value);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Set %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  return true;
}

bool HkLightControllerNode::getEnumValue(const std::string& key, int& value)
{
  MVCC_ENUMVALUE stEnumValue;
  memset(&stEnumValue, 0, sizeof(MVCC_ENUMVALUE));
  int nRet = MV_CC_GetEnumValue(interface_handle_, key.c_str(), &stEnumValue);
  if (MV_OK != nRet) {
    RCLCPP_DEBUG(this->get_logger(), "Get %s failed! nRet [0x%x]", key.c_str(), nRet);
    return false;
  }
  value = static_cast<int>(stEnumValue.nCurValue);
  return true;
}

bool HkLightControllerNode::getEnumEntrySymbolic(const std::string& key, int value, std::string& symbolic)
{
  MVCC_ENUMENTRY stEnumentryInfo;
  memset(&stEnumentryInfo, 0, sizeof(MVCC_ENUMENTRY));
  stEnumentryInfo.nValue = value;
  int nRet = MV_CC_GetEnumEntrySymbolic(interface_handle_, key.c_str(), &stEnumentryInfo);
  if (MV_OK != nRet) {
    return false;
  }
  symbolic = std::string((char*)stEnumentryInfo.chSymbolic);
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<HkLightControllerNode>();
  
  // Check if initialization was successful
  if (!node->interface_initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), 
                 "Light controller node created but interface not initialized. "
                 "Node will continue running but control functions will not work.");
    RCLCPP_ERROR(rclcpp::get_logger("main"), 
                 "You can still use the node to check connection status via topics.");
  }
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

