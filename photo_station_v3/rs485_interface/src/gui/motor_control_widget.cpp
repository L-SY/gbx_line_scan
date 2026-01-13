#include "rs485_interface/gui/motor_control_widget.hpp"

MotorControlWidget::MotorControlWidget(
  const QString & motor_name,
  MotorType type,
  QWidget * parent)
: QWidget(parent),
  motor_name_(motor_name),
  motor_type_(type)
{
}

void MotorControlWidget::logMessage(const std::string & message)
{
  emit statusMessage(QString::fromStdString(message));
}


