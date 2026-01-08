#include "rs485_interface/motor/zd_motor/zd_motor_test_gui.hpp"
#include <QApplication>
#include <iostream>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);

  // Set application metadata
  app.setApplicationName("ZD Motor Control Test");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("RS485 Interface");

  ZdMotorTestGUI gui;
  gui.show();

  return app.exec();
}

