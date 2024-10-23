#include "tobas_control_system/mission_planner/commands/land.hpp"

namespace gui
{
namespace control_system
{
LandWidget::LandWidget()
{
}

const char* LandWidget::name() const
{
  return "Land";
}

BaseCommandData::SharedPtr LandWidget::data() const
{
  const auto res = std::make_shared<LandData>();
  return res;
}
}  // namespace control_system
}  // namespace gui
