#include "tobas_control_system/mission_planner/commands/return_to_launch.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
ReturnToLaunchWidget::ReturnToLaunchWidget()
{
  min_alt_ = new field::RtlMinAltitudeWidget();
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

  addField(min_alt_, true);
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

const char* ReturnToLaunchWidget::name() const
{
  return "Return to Launch";
}

double ReturnToLaunchWidget::minAltitude() const
{
  return min_alt_->getValue();
}

double ReturnToLaunchWidget::maxHorizontalVelocity() const
{
  return getValue(max_hor_vel_);
}

double ReturnToLaunchWidget::maxHorizontalAccel() const
{
  return getValue(max_hor_acc_);
}

double ReturnToLaunchWidget::maxVerticalVelocity() const
{
  return getValue(max_ver_vel_);
}

double ReturnToLaunchWidget::maxHorizontalJerk() const
{
  return getValue(max_hor_jerk_);
}

double ReturnToLaunchWidget::maxVerticalAccel() const
{
  return getValue(max_ver_acc_);
}

double ReturnToLaunchWidget::maxVerticalJerk() const
{
  return getValue(max_ver_jerk_);
}

double ReturnToLaunchWidget::maxHeadingRate() const
{
  return getValue(max_head_rate_);
}

double ReturnToLaunchWidget::maxHeadingAccel() const
{
  return getValue(max_head_acc_);
}

double ReturnToLaunchWidget::acceptanceRadius() const
{
  return getValue(acceptance_radius_);
}

double ReturnToLaunchWidget::altitudeTolerance() const
{
  return getValue(altitude_tolerance_);
}

void ReturnToLaunchWidget::minAltitude(double value)
{
  min_alt_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalVelocity(double value)
{
  max_hor_vel_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalAccel(double value)
{
  max_hor_acc_->setValue(value);
}

void ReturnToLaunchWidget::maxHorizontalJerk(double value)
{
  max_hor_jerk_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalVelocity(double value)
{
  max_ver_vel_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalAccel(double value)
{
  max_ver_acc_->setValue(value);
}

void ReturnToLaunchWidget::maxVerticalJerk(double value)
{
  max_ver_jerk_->setValue(value);
}

void ReturnToLaunchWidget::maxHeadingRate(double value)
{
  max_head_rate_->setValue(value);
}

void ReturnToLaunchWidget::maxHeadingAccel(double value)
{
  max_head_acc_->setValue(value);
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
}  // namespace tobas
