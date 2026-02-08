#include "tobas_control_system/mission_planner/commands/land.hpp"

namespace gui
{
namespace ctrl
{
LandWidget::LandWidget()
{
  speed_ = new field::LandSpeedWidget();

  addField(speed_);
}

const char* LandWidget::name() const
{
  return "Land";
}

double LandWidget::speed() const
{
  return speed_->value();
}

void LandWidget::speed(double value)
{
  speed_->setValue(value);
}
}  // namespace ctrl
}  // namespace gui
