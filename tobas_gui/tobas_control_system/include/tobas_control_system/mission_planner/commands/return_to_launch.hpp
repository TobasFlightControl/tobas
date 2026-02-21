#pragma once

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/max_heading_accel.hpp"
#include "../fields/max_heading_rate.hpp"
#include "../fields/max_horizontal_accel.hpp"
#include "../fields/max_horizontal_jerk.hpp"
#include "../fields/max_horizontal_velocity.hpp"
#include "../fields/max_vertical_accel.hpp"
#include "../fields/max_vertical_jerk.hpp"
#include "../fields/max_vertical_velocity.hpp"
#include "../fields/rtl_min_altitude.hpp"
#include "./base.hpp"

namespace gui
{
namespace ctrl
{
class ReturnToLaunchWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = ReturnToLaunchWidget;
  using super = BaseCommandWidget;

public:
  explicit ReturnToLaunchWidget();

  const char* name() const override;

  double minAltitude() const;
  double maxHorizontalVelocity() const;
  double maxHorizontalAccel() const;
  double maxHorizontalJerk() const;
  double maxVerticalVelocity() const;
  double maxVerticalAccel() const;
  double maxVerticalJerk() const;
  double maxHeadingRate() const;
  double maxHeadingAccel() const;
  double acceptanceRadius() const;
  double altitudeTolerance() const;

  void minAltitude(double value);
  void maxHorizontalVelocity(double value);
  void maxHorizontalAccel(double value);
  void maxHorizontalJerk(double value);
  void maxVerticalVelocity(double value);
  void maxVerticalAccel(double value);
  void maxVerticalJerk(double value);
  void maxHeadingRate(double value);
  void maxHeadingAccel(double value);
  void acceptanceRadius(double value);
  void altitudeTolerance(double value);

private:
  field::RtlMinAltitudeWidget* min_alt_;
  field::MaxHorizontalVelocityWidget* max_hor_vel_;
  field::MaxHorizontalAccelWidget* max_hor_acc_;
  field::MaxHorizontalJerkWidget* max_hor_jerk_;
  field::MaxVerticalVelocityWidget* max_ver_vel_;
  field::MaxVerticalAccelWidget* max_ver_acc_;
  field::MaxVerticalJerkWidget* max_ver_jerk_;
  field::MaxHeadingRateWidget* max_head_rate_;
  field::MaxHeadingAccelWidget* max_head_acc_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
};
}  // namespace ctrl
}  // namespace gui
