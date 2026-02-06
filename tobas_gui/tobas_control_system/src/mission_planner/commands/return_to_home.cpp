#include "tobas_control_system/mission_planner/commands/return_to_launch.hpp"

namespace gui
{
namespace ctrl
{
ReturnToLaunchWidget::ReturnToLaunchWidget()
{
  min_alt_ = new field::RtlMinAltitudeWidget();
  max_hor_vel_ = new field::MaxHorizontalVelocityWidget();
  max_ver_vel_ = new field::MaxVerticalVelocityWidget();
  max_hor_acc_ = new field::MaxHorizontalAccelWidget();
  max_ver_acc_ = new field::MaxVerticalAccelWidget();
  max_hor_jerk_ = new field::MaxHorizontalJerkWidget();
  max_ver_jerk_ = new field::MaxVerticalJerkWidget();
  acceptance_radius_ = new field::AcceptanceRadiusWidget();
  altitude_tolerance_ = new field::AltitudeToleranceWidget();

  addField(min_alt_);
  addField(max_hor_vel_);
  addField(max_ver_vel_);
  addField(max_hor_acc_);
  addField(max_ver_acc_);
  addField(max_hor_jerk_);
  addField(max_ver_jerk_);
  addField(acceptance_radius_);
  addField(altitude_tolerance_);
}

const char* ReturnToLaunchWidget::name() const
{
  return "Return to Launch";
}

double ReturnToLaunchWidget::minAltitude() const
{
  return min_alt_->value();
}

double ReturnToLaunchWidget::maxHorizontalVelocity() const
{
  return max_hor_vel_->value();
}

double ReturnToLaunchWidget::maxVerticalVelocity() const
{
  return max_ver_vel_->value();
}

double ReturnToLaunchWidget::maxHorizontalAccel() const
{
  return max_hor_acc_->value();
}

double ReturnToLaunchWidget::maxVerticalAccel() const
{
  return max_ver_acc_->value();
}

double ReturnToLaunchWidget::maxHorizontalJerk() const
{
  return max_hor_jerk_->value();
}

double ReturnToLaunchWidget::maxVerticalJerk() const
{
  return max_ver_jerk_->value();
}

double ReturnToLaunchWidget::acceptanceRadius() const
{
  return acceptance_radius_->value();
}

double ReturnToLaunchWidget::altitudeTolerance() const
{
  return altitude_tolerance_->value();
}

void ReturnToLaunchWidget::minAltitude(double value)
{
  min_alt_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalVelocity(double value)
{
  max_hor_vel_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalVelocity(double value)
{
  max_ver_vel_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalAccel(double value)
{
  max_hor_acc_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalAccel(double value)
{
  max_ver_acc_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalJerk(double value)
{
  max_hor_jerk_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalJerk(double value)
{
  max_ver_jerk_->setValue(value);
}

void ReturnToLaunchWidget::acceptanceRadius(double value)
{
  acceptance_radius_->setValue(value);
}

void ReturnToLaunchWidget::altitudeTolerance(double value)
{
  altitude_tolerance_->setValue(value);
}
}  // namespace ctrl
}  // namespace gui
