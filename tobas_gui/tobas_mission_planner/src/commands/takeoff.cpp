

#include "tobas_mission_planner/commands/takeoff.hpp"

namespace gui
{
namespace mission_planner
{
TakeoffWidget::TakeoffWidget()
{
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();
  duration_ = new field::DurationWidget();

  addField(altitude_);
  addField(altitude_frame_);
  addField(altitude_tolerance_);
  addField(duration_);
}

const char* TakeoffWidget::name() const
{
  return "Takeoff";
}

BaseCommandData::SharedPtr TakeoffWidget::data() const
{
  const auto res = std::make_shared<TakeoffData>();
  res->altitude = altitude_->value();
  res->altitude_frame = altitude_frame_->value();
  res->altitude_tolerance = altitude_tolerance_->value();
  res->duration = duration_->value();
  return res;
}
}  // namespace mission_planner
}  // namespace gui
