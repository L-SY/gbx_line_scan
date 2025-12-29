#include "hk_light_controller/light_controller_gui.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  QApplication app(argc, argv);
  
  // Set application metadata
  app.setApplicationName("Light Controller GUI");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("HK Devices");
  
  LightControllerGUI gui;
  gui.show();
  
  int result = app.exec();
  
  // Shutdown ROS2
  rclcpp::shutdown();
  
  return result;
}

