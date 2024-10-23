#include "tobas_control_system/mission_planner/commands/takeoff.hpp"

namespace gui
{
namespace control_system
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
  res->altitude = altitude();
  res->altitude_frame = altitudeFrame();
  res->altitude_tolerance = altitudeTolerance();
  res->duration = duration();
  return res;
}

double TakeoffWidget::altitude() const
{
  return altitude_->value();
}

altitude_frame_t TakeoffWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double TakeoffWidget::altitudeTolerance() const
{
  return altitude_tolerance_->value();
}

double TakeoffWidget::duration() const
{
  return duration_->value();
}
}  // namespace control_system
}  // namespace gui
