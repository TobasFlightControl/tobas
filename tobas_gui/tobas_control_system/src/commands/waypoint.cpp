

#include "tobas_control_system/commands/waypoint.hpp"

namespace gui
{
namespace control_system
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
  res->latitude = latitude();
  res->longitude = longitude();
  res->altitude = altitude();
  res->altitude_frame = altitudeFrame();
  res->acceptance_radius = acceptanceRadius();
  res->duration = duration();
  return res;
}

double WaypointWidget::latitude() const
{
  return latitude_->value();
}

double WaypointWidget::longitude() const
{
  return longitude_->value();
}

double WaypointWidget::altitude() const
{
  return altitude_->value();
}

altitude_frame_t WaypointWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double WaypointWidget::acceptanceRadius() const
{
  return acceptance_radius_->value();
}

double WaypointWidget::duration() const
{
  return duration_->value();
}

void WaypointWidget::latitude(double value)
{
  latitude_->setValue(value);
}

void WaypointWidget::longitude(double value)
{
  longitude_->setValue(value);
}

void WaypointWidget::altitude(double value)
{
  altitude_->setValue(value);
}

void WaypointWidget::altitudeFrame(altitude_frame_t value)
{
  altitude_frame_->setValue(value);
}

void WaypointWidget::acceptanceRadius(double value)
{
  acceptance_radius_->setValue(value);
}

void WaypointWidget::duration(double value)
{
  duration_->setValue(value);
}
}  // namespace control_system
}  // namespace gui
