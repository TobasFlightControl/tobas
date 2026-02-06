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
  addField(max_speed_);
  addField(max_accel_);
  addField(max_jerk_);
  addField(altitude_tolerance_);
}

const char* TakeoffWidget::name() const
{
  return "Takeoff";
}

double TakeoffWidget::altitude() const
{
  return altitude_->value();
}

tobas::mission::AltitudeFrame TakeoffWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double TakeoffWidget::maxSpeed() const
{
  return max_speed_->value();
}

double TakeoffWidget::maxAccel() const
{
  return max_accel_->value();
}

double TakeoffWidget::maxJerk() const
{
  return max_jerk_->value();
}

double TakeoffWidget::altitudeTolerance() const
{
  return altitude_tolerance_->value();
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
