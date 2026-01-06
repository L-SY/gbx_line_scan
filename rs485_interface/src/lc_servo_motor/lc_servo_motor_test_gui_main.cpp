#include "rs485_interface/lc_servo_motor/lc_servo_motor_test_gui.hpp"
#include <QApplication>
#include <iostream>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);

  // Set application metadata
  app.setApplicationName("LC Servo Motor Speed Control Test");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("RS485 Interface");

  LcServoMotorTestGUI gui;
  gui.show();

  return app.exec();
}

