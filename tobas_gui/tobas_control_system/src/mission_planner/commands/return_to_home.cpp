#include "tobas_control_system/mission_planner/commands/return_to_home.hpp"

namespace gui
{
namespace ctrl
{
ReturnToHomeWidget::ReturnToHomeWidget()
{
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

const char* ReturnToHomeWidget::name() const
{
  return "Return to Home";
}

BaseCommandData::SharedPtr ReturnToHomeWidget::data() const
{
  const auto res = std::make_shared<ReturnToHomeData>();
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

double ReturnToHomeWidget::altitude() const
{
  return altitude_->value();
}

AltitudeFrame ReturnToHomeWidget::altitudeFrame() const
{
  return altitude_frame_->value();
}

double ReturnToHomeWidget::maxHorizontalVelocity() const
{
  return max_hor_vel_->value();
}

double ReturnToHomeWidget::maxVerticalVelocity() const
{
  return max_ver_vel_->value();
}

double ReturnToHomeWidget::maxHorizontalAccel() const
{
  return max_hor_acc_->value();
}

double ReturnToHomeWidget::maxVerticalAccel() const
{
  return max_ver_acc_->value();
}

double ReturnToHomeWidget::maxHorizontalJerk() const
{
  return max_hor_jerk_->value();
}

double ReturnToHomeWidget::maxVerticalJerk() const
{
  return max_ver_jerk_->value();
}

double ReturnToHomeWidget::acceptanceRadius() const
{
  return acceptance_radius_->value();
}

double ReturnToHomeWidget::altitudeTolerance() const
{
  return altitude_tolerance_->value();
}

void ReturnToHomeWidget::altitude(double value)
{
  altitude_->setValue(value);
}

void ReturnToHomeWidget::altitudeFrame(AltitudeFrame value)
{
  altitude_frame_->setValue(value);
}

void ReturnToHomeWidget::maxHorizontalVelocity(double value)
{
  max_hor_vel_->setValue(value);
}

void ReturnToHomeWidget::maxVerticalVelocity(double value)
{
  max_ver_vel_->setValue(value);
}

void ReturnToHomeWidget::maxHorizontalAccel(double value)
{
  max_hor_acc_->setValue(value);
}

void ReturnToHomeWidget::maxVerticalAccel(double value)
{
  max_ver_acc_->setValue(value);
}

void ReturnToHomeWidget::maxHorizontalJerk(double value)
{
  max_hor_jerk_->setValue(value);
}

void ReturnToHomeWidget::maxVerticalJerk(double value)
{
  max_ver_jerk_->setValue(value);
}

void ReturnToHomeWidget::acceptanceRadius(double value)
{
  acceptance_radius_->setValue(value);
}

void ReturnToHomeWidget::altitudeTolerance(double value)
{
  altitude_tolerance_->setValue(value);
}
}  // namespace ctrl
}  // namespace gui
