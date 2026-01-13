#include "image_viewer/image_viewer_widget.hpp"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  QApplication app(argc, argv);
  
  // Set application metadata
  app.setApplicationName("Image Viewer");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("Image Viewer");
  
  ImageViewerWidget viewer;
  viewer.show();
  
  int result = app.exec();
  
  // Shutdown ROS2
  rclcpp::shutdown();
  
  return result;
}

