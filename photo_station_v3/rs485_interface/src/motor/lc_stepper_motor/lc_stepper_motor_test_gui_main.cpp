#include "rs485_interface/motor/lc_stepper_motor/lc_stepper_motor_test_gui.hpp"
#include <QApplication>
#include <iostream>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);

  // Set application metadata
  app.setApplicationName("LC Stepper Motor Test");
  app.setApplicationVersion("1.0");
  app.setOrganizationName("RS485 Interface");

  LcStepperMotorTestGUI gui;
  gui.show();

  return app.exec();
}

