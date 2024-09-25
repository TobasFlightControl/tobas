

#include "tobas_mission_planner/commands/return_to_home.hpp"

namespace gui
{
namespace mission_planner
{
ReturnToHomeWidget::ReturnToHomeWidget()
{
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  duration_ = new field::DurationWidget();

  addField(altitude_);
  addField(altitude_frame_);
  addField(acceptance_radius_);
  addField(duration_);
}

const char* ReturnToHomeWidget::name() const
{
  return "Return to Home";
}

BaseCommandData::SharedPtr ReturnToHomeWidget::data() const
{
  const auto res = std::make_shared<ReturnToHomeData>();
  res->altitude = altitude_->value();
  res->altitude_frame = altitude_frame_->value();
  res->acceptance_radius = acceptance_radius_->value();
  res->duration = duration_->value();
  return res;
}
}  // namespace mission_planner
}  // namespace gui
