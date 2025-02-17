#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/manual.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
const char* SpeedLimitWidget_Manual::name() const
{
  return "Set Manually";
}

void SpeedLimitWidget_Manual::onInit()
{
  spinbox_->setDecimals(0);
  spinbox_->setMinimum(0);
  spinbox_->setValue(10000);
  spinbox_->setSuffix(" rpm");
}

bool SpeedLimitWidget_Manual::isValid()
{
  return true;
}

double SpeedLimitWidget_Manual::maxRotSpeed() const
{
  const auto rpm = spinbox_->value();
  return tobas_std::rpm2rps(rpm);
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
