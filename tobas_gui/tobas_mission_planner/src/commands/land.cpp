

#include "tobas_mission_planner/commands/land.hpp"

namespace gui
{
namespace mission_planner
{
LandWidget::LandWidget()
{
  duration_ = new field::DurationWidget();

  addField(duration_);
}

const char* LandWidget::name() const
{
  return "Land";
}

BaseCommandData::SharedPtr LandWidget::data() const
{
  const auto res = std::make_shared<LandData>();
  res->duration = duration_->value();
  return res;
}
}  // namespace mission_planner
}  // namespace gui
