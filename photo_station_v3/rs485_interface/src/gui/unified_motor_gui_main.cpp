#include "rs485_interface/gui/unified_motor_gui.hpp"
#include <QApplication>

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);
  
  UnifiedMotorGUI gui;
  gui.show();
  
  return app.exec();
}


