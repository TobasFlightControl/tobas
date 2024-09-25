

#include "tobas_mission_planner/commands/waypoint.hpp"

namespace gui
{
namespace mission_planner
{
WaypointWidget::WaypointWidget()
{
  latitude_ = new field::LatitudeWidget();
  longitude_ = new field::LongitudeWidget();
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  duration_ = new field::DurationWidget();

  addField(latitude_);
  addField(longitude_);
  addField(altitude_);
  addField(altitude_frame_);
  addField(acceptance_radius_);
  addField(duration_);
}

const char* WaypointWidget::name() const
{
  return "Waypoint";
}

BaseCommandData::SharedPtr WaypointWidget::data() const
{
  const auto res = std::make_shared<WaypointData>();
  res->latitude = latitude_->value();
  res->longitude = longitude_->value();
  res->altitude = altitude_->value();
  res->altitude_frame = altitude_frame_->value();
  res->acceptance_radius = acceptance_radius_->value();
  res->duration = duration_->value();
  return res;
}
}  // namespace mission_planner
}  // namespace gui
