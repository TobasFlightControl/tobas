#include "tobas_control_system/mission_planner/commands/waypoint.hpp"

namespace gui
{
namespace ctrl
{
WaypointWidget::WaypointWidget()
{
  latitude_ = new field::LatitudeWidget();
  longitude_ = new field::LongitudeWidget();
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  max_hor_vel_ = new field::MaxHorizontalVelocityWidget();
  max_ver_vel_ = new field::MaxVerticalVelocityWidget();
  max_hor_acc_ = new field::MaxHorizontalAccelWidget();
  max_ver_acc_ = new field::MaxVerticalAccelWidget();
  max_hor_jerk_ = new field::MaxHorizontalJerkWidget();
  max_ver_jerk_ = new field::MaxVerticalJerkWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(latitude_);
  addField(longitude_);
  addField(altitude_);
  addField(altitude_frame_);
  addField(max_hor_vel_);
  addField(max_ver_vel_);
  addField(max_hor_acc_);
  addField(max_ver_acc_);
  addField(max_hor_jerk_);
  addField(max_ver_jerk_);
  addField(acceptance_radius_);
  addField(altitude_tolerance_);
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
  res->max_horizontal_velocity = maxHorizontalVelocity();
  res->max_vertical_velocity = maxVerticalVelocity();
  res->max_horizontal_accel = maxHorizontalAccel();
  res->max_vertical_accel = maxVerticalAccel();
  res->max_horizontal_jerk = maxHorizontalJerk();
  res->max_vertical_jerk = maxVerticalJerk();
  res->acceptance_radius = acceptanceRadius();
  res->altitude_tolerance = altitudeTolerance();
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

AltitudeFrame WaypointWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double WaypointWidget::maxHorizontalVelocity() const
{
  return max_hor_vel_->value();
}

double WaypointWidget::maxVerticalVelocity() const
{
  return max_ver_vel_->value();
}

double WaypointWidget::maxHorizontalAccel() const
{
  return max_hor_acc_->value();
}

double WaypointWidget::maxVerticalAccel() const
{
  return max_ver_acc_->value();
}

double WaypointWidget::maxHorizontalJerk() const
{
  return max_hor_jerk_->value();
}

double WaypointWidget::maxVerticalJerk() const
{
  return max_ver_jerk_->value();
}

double WaypointWidget::acceptanceRadius() const
{
  return acceptance_radius_->value();
}

double WaypointWidget::altitudeTolerance() const
{
  return altitude_tolerance_->value();
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

void WaypointWidget::altitudeFrame(AltitudeFrame value)
{
  altitude_frame_->setValue(value);
}

void WaypointWidget::maxHorizontalVelocity(double value)
{
  max_hor_vel_->setValue(value);
}

void WaypointWidget::maxVerticalVelocity(double value)
{
  max_ver_vel_->setValue(value);
}

void WaypointWidget::maxHorizontalAccel(double value)
{
  max_hor_acc_->setValue(value);
}

void WaypointWidget::maxVerticalAccel(double value)
{
  max_ver_acc_->setValue(value);
}

void WaypointWidget::maxHorizontalJerk(double value)
{
  max_hor_jerk_->setValue(value);
}

void WaypointWidget::maxVerticalJerk(double value)
{
  max_ver_jerk_->setValue(value);
}

void WaypointWidget::acceptanceRadius(double value)
{
  acceptance_radius_->setValue(value);
}

void WaypointWidget::altitudeTolerance(double value)
{
  altitude_tolerance_->setValue(value);
}
}  // namespace ctrl
}  // namespace gui
