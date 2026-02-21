#include "tobas_control_system/mission_planner/commands/takeoff.hpp"

namespace gui
{
namespace ctrl
{
TakeoffWidget::TakeoffWidget()
{
  altitude_ = new field::AltitudeWidget();
  altitude_frame_ = new field::AltitudeFrameWidget();
  max_speed_ = new field::TakeoffMaxSpeedWidget();
  max_accel_ = new field::TakeoffMaxAccelWidget();
  max_jerk_ = new field::TakeoffMaxJerkWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(altitude_);
  addField(altitude_frame_);
  addField(max_speed_, true);
  addField(max_accel_, true);
  addField(max_jerk_, true);
  addField(altitude_tolerance_);
}

const char* TakeoffWidget::name() const
{
  return "Takeoff";
}

double TakeoffWidget::altitude() const
{
  return getValue(altitude_);
}

tobas::mission::AltitudeFrame TakeoffWidget::altitudeFrame() const
{
  return getValue(altitude_frame_);
}

double TakeoffWidget::maxSpeed() const
{
  return getValue(max_speed_);
}

double TakeoffWidget::maxAccel() const
{
  return getValue(max_accel_);
}

double TakeoffWidget::maxJerk() const
{
  return getValue(max_jerk_);
}

double TakeoffWidget::altitudeTolerance() const
{
  return getValue(altitude_tolerance_);
}

void TakeoffWidget::altitude(double value)
{
  altitude_->setValue(value);
}

void TakeoffWidget::altitudeFrame(tobas::mission::AltitudeFrame value)
{
  altitude_frame_->setValue(value);
}

void TakeoffWidget::maxSpeed(double value)
{
  max_speed_->setValue(value);
}

void TakeoffWidget::maxAccel(double value)
{
  max_accel_->setValue(value);
}

void TakeoffWidget::maxJerk(double value)
{
  max_jerk_->setValue(value);
}

void TakeoffWidget::altitudeTolerance(double value)
{
  altitude_tolerance_->setValue(value);
}
}  // namespace ctrl
}  // namespace gui
