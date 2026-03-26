#include "tobas_control_system/mission_planner/commands/waypoint.hpp"

namespace tobas
{
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
  max_hor_acc_ = new field::MaxHorizontalAccelWidget();
  max_hor_jerk_ = new field::MaxHorizontalJerkWidget();
  max_ver_vel_ = new field::MaxVerticalVelocityWidget();
  max_ver_acc_ = new field::MaxVerticalAccelWidget();
  max_ver_jerk_ = new field::MaxVerticalJerkWidget();
  max_head_rate_ = new field::MaxHeadingRateWidget();
  max_head_acc_ = new field::MaxHeadingAccelWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(latitude_);
  addField(longitude_);
  addField(altitude_);
  addField(altitude_frame_);
  addField(max_hor_vel_, true);
  addField(max_hor_acc_, true);
  addField(max_hor_jerk_, true);
  addField(max_ver_vel_, true);
  addField(max_ver_acc_, true);
  addField(max_ver_jerk_, true);
  addField(max_head_rate_, true);
  addField(max_head_acc_, true);
  addField(acceptance_radius_);
  addField(altitude_tolerance_);
}

const char* WaypointWidget::name() const
{
  return "Waypoint";
}

double WaypointWidget::latitude() const
{
  return getValue(latitude_);
}

double WaypointWidget::longitude() const
{
  return getValue(longitude_);
}

double WaypointWidget::altitude() const
{
  return getValue(altitude_);
}

tobas::mission::AltitudeFrame WaypointWidget::altitudeFrame() const
{
  return getValue(altitude_frame_);
}

double WaypointWidget::maxHorizontalVelocity() const
{
  return getValue(max_hor_vel_);
}

double WaypointWidget::maxHorizontalAccel() const
{
  return getValue(max_hor_acc_);
}

double WaypointWidget::maxVerticalVelocity() const
{
  return getValue(max_ver_vel_);
}

double WaypointWidget::maxHorizontalJerk() const
{
  return getValue(max_hor_jerk_);
}

double WaypointWidget::maxVerticalAccel() const
{
  return getValue(max_ver_acc_);
}

double WaypointWidget::maxVerticalJerk() const
{
  return getValue(max_ver_jerk_);
}

double WaypointWidget::maxHeadingRate() const
{
  return getValue(max_head_rate_);
}

double WaypointWidget::maxHeadingAccel() const
{
  return getValue(max_head_acc_);
}

double WaypointWidget::acceptanceRadius() const
{
  return getValue(acceptance_radius_);
}

double WaypointWidget::altitudeTolerance() const
{
  return getValue(altitude_tolerance_);
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

void WaypointWidget::altitudeFrame(tobas::mission::AltitudeFrame value)
{
  altitude_frame_->setValue(value);
}

void WaypointWidget::maxHorizontalVelocity(double value)
{
  max_hor_vel_->setValue(value);
}

void WaypointWidget::maxHorizontalAccel(double value)
{
  max_hor_acc_->setValue(value);
}

void WaypointWidget::maxHorizontalJerk(double value)
{
  max_hor_jerk_->setValue(value);
}

void WaypointWidget::maxVerticalVelocity(double value)
{
  max_ver_vel_->setValue(value);
}

void WaypointWidget::maxVerticalAccel(double value)
{
  max_ver_acc_->setValue(value);
}

void WaypointWidget::maxVerticalJerk(double value)
{
  max_ver_jerk_->setValue(value);
}

void WaypointWidget::maxHeadingRate(double value)
{
  max_head_rate_->setValue(value);
}

void WaypointWidget::maxHeadingAccel(double value)
{
  max_head_acc_->setValue(value);
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
}  // namespace tobas
