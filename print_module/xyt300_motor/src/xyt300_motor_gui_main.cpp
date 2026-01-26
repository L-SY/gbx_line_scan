#include "xyt300_motor/xyt300_motor_gui.hpp"
#include <QApplication>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);
  
  Xyt300MotorGUI gui;
  gui.show();
  
  return app.exec();
}
