

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
  res->altitude = altitude();
  res->altitude_frame = altitudeFrame();
  res->acceptance_radius = acceptanceRadius();
  res->duration = duration();
  return res;
}

double ReturnToHomeWidget::altitude() const
{
  return altitude_->value();
}

altitude_frame_t ReturnToHomeWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double ReturnToHomeWidget::acceptanceRadius() const
{
  return acceptance_radius_->value();
}

double ReturnToHomeWidget::duration() const
{
  return duration_->value();
}
}  // namespace mission_planner
}  // namespace gui
