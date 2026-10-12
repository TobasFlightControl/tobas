// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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
  stop_at_waypoint_ = new field::StopAtWaypointWidget();
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
  addField(stop_at_waypoint_);
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

  updateToleranceFieldsEnabled();
}

const char* WaypointWidget::name() const
{
  return "Waypoint";
}

mission::Waypoint WaypointWidget::dump() const
{
  mission::Waypoint res;

  res.latitude = latitude();
  res.longitude = longitude();
  res.altitude = altitude();
  res.altitude_frame = altitudeFrame();
  res.auto_heading = true;  // TODO
  res.stop_at_waypoint = stopAtWaypoint();
  res.max_horizontal_velocity = maxHorizontalVelocity();
  res.max_horizontal_accel = maxHorizontalAccel();
  res.max_horizontal_jerk = maxHorizontalJerk();
  res.max_vertical_velocity = maxVerticalVelocity();
  res.max_vertical_accel = maxVerticalAccel();
  res.max_vertical_jerk = maxVerticalJerk();
  res.max_heading_rate = maxHeadingRate();
  res.max_heading_accel = maxHeadingAccel();
  res.acceptance_radius = acceptanceRadius();
  res.altitude_tolerance = altitudeTolerance();
  res.timeout = 0.0;  // TODO

  return res;
}

void WaypointWidget::load(const mission::Waypoint& _src)
{
  latitude(_src.latitude);
  longitude(_src.longitude);
  altitude(_src.altitude);
  altitudeFrame(_src.altitude_frame);
  stopAtWaypoint(_src.stop_at_waypoint);
  maxHorizontalVelocity(_src.max_horizontal_velocity);
  maxHorizontalAccel(_src.max_horizontal_accel);
  maxHorizontalJerk(_src.max_horizontal_jerk);
  maxVerticalVelocity(_src.max_vertical_velocity);
  maxVerticalAccel(_src.max_vertical_accel);
  maxVerticalJerk(_src.max_vertical_jerk);
  maxHeadingRate(_src.max_heading_rate);
  maxHeadingAccel(_src.max_heading_accel);
  acceptanceRadius(_src.acceptance_radius);
  altitudeTolerance(_src.altitude_tolerance);
}

double WaypointWidget::latitude() const
{
  return latitude_->getValue();
}

double WaypointWidget::longitude() const
{
  return longitude_->getValue();
}

double WaypointWidget::altitude() const
{
  return altitude_->getValue();
}

mission::AltitudeFrame WaypointWidget::altitudeFrame() const
{
  return altitude_frame_->getValue();
}

bool WaypointWidget::stopAtWaypoint() const
{
  return stop_at_waypoint_->getValue();
}

double WaypointWidget::maxHorizontalVelocity() const
{
  return getValueOrDefault(max_hor_vel_);
}

double WaypointWidget::maxHorizontalAccel() const
{
  return getValueOrDefault(max_hor_acc_);
}

double WaypointWidget::maxVerticalVelocity() const
{
  return getValueOrDefault(max_ver_vel_);
}

double WaypointWidget::maxHorizontalJerk() const
{
  return getValueOrDefault(max_hor_jerk_);
}

double WaypointWidget::maxVerticalAccel() const
{
  return getValueOrDefault(max_ver_acc_);
}

double WaypointWidget::maxVerticalJerk() const
{
  return getValueOrDefault(max_ver_jerk_);
}

double WaypointWidget::maxHeadingRate() const
{
  return getValueOrDefault(max_head_rate_);
}

double WaypointWidget::maxHeadingAccel() const
{
  return getValueOrDefault(max_head_acc_);
}

double WaypointWidget::acceptanceRadius() const
{
  return getValueOrDefault(acceptance_radius_);
}

double WaypointWidget::altitudeTolerance() const
{
  return getValueOrDefault(altitude_tolerance_);
}

void WaypointWidget::latitude(double _value)
{
  latitude_->setValue(_value);
}

void WaypointWidget::longitude(double _value)
{
  longitude_->setValue(_value);
}

void WaypointWidget::altitude(double _value)
{
  altitude_->setValue(_value);
}

void WaypointWidget::altitudeFrame(mission::AltitudeFrame _value)
{
  altitude_frame_->setValue(_value);
}

void WaypointWidget::stopAtWaypoint(bool _value)
{
  stop_at_waypoint_->setValue(_value);
}

void WaypointWidget::maxHorizontalVelocity(double _value)
{
  setValueOrDefault(max_hor_vel_, _value);
}

void WaypointWidget::maxHorizontalAccel(double _value)
{
  setValueOrDefault(max_hor_acc_, _value);
}

void WaypointWidget::maxHorizontalJerk(double _value)
{
  setValueOrDefault(max_hor_jerk_, _value);
}

void WaypointWidget::maxVerticalVelocity(double _value)
{
  setValueOrDefault(max_ver_vel_, _value);
}

void WaypointWidget::maxVerticalAccel(double _value)
{
  setValueOrDefault(max_ver_acc_, _value);
}

void WaypointWidget::maxVerticalJerk(double _value)
{
  setValueOrDefault(max_ver_jerk_, _value);
}

void WaypointWidget::maxHeadingRate(double _value)
{
  setValueOrDefault(max_head_rate_, _value);
}

void WaypointWidget::maxHeadingAccel(double _value)
{
  setValueOrDefault(max_head_acc_, _value);
}

void WaypointWidget::acceptanceRadius(double _value)
{
  setValueOrDefault(acceptance_radius_, _value);
}

void WaypointWidget::altitudeTolerance(double _value)
{
  setValueOrDefault(altitude_tolerance_, _value);
}

bool WaypointWidget::isSplineSegmentEnd() const
{
  return spline_segment_end_;
}

void WaypointWidget::setSplineSegmentEnd(bool _value)
{
  spline_segment_end_ = _value;
  updateToleranceFieldsEnabled();
}

void WaypointWidget::updateToleranceFieldsEnabled()
{
  acceptance_radius_->setEnabled(spline_segment_end_);
  altitude_tolerance_->setEnabled(spline_segment_end_);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
