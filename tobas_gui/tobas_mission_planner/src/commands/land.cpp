

#include "tobas_mission_planner/commands/land.hpp"

namespace gui
{
namespace mission_planner
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
}  // namespace mission_planner
}  // namespace gui
