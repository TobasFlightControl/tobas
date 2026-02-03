#pragma once

#include "../fields/acceptance_radius.hpp"
#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/max_horizontal_accel.hpp"
#include "../fields/max_horizontal_jerk.hpp"
#include "../fields/max_horizontal_velocity.hpp"
#include "../fields/max_vertical_accel.hpp"
#include "../fields/max_vertical_jerk.hpp"
#include "../fields/max_vertical_velocity.hpp"
#include "./base.hpp"

namespace gui
{
namespace ctrl
{
struct ReturnToHomeData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<ReturnToHomeData>;

  double altitude;  // [m]
  AltitudeFrame altitude_frame;
  double max_horizontal_velocity;  // [m/s]
  double max_vertical_velocity;    // [m/s]
  double max_horizontal_accel;     // [m/s^2]
  double max_vertical_accel;       // [m/s^2]
  double max_horizontal_jerk;      // [m/s^3]
  double max_vertical_jerk;        // [m/s^3]
  double acceptance_radius;        // [m]
  double altitude_tolerance;       // [m]

  Command type() const override
  {
    return Command::kReturnToHome;
  }
};

class ReturnToHomeWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = ReturnToHomeWidget;
  using super = BaseCommandWidget;

public:
  explicit ReturnToHomeWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

  double altitude() const;
  AltitudeFrame altitudeFrame() const;
  double maxHorizontalVelocity() const;
  double maxVerticalVelocity() const;
  double maxHorizontalAccel() const;
  double maxVerticalAccel() const;
  double maxHorizontalJerk() const;
  double maxVerticalJerk() const;
  double acceptanceRadius() const;
  double altitudeTolerance() const;

  void altitude(double value);
  void altitudeFrame(AltitudeFrame value);
  void maxHorizontalVelocity(double value);
  void maxVerticalVelocity(double value);
  void maxHorizontalAccel(double value);
  void maxVerticalAccel(double value);
  void maxHorizontalJerk(double value);
  void maxVerticalJerk(double value);
  void acceptanceRadius(double value);
  void altitudeTolerance(double value);

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::MaxHorizontalVelocityWidget* max_hor_vel_;
  field::MaxVerticalVelocityWidget* max_ver_vel_;
  field::MaxHorizontalAccelWidget* max_hor_acc_;
  field::MaxVerticalAccelWidget* max_ver_acc_;
  field::MaxHorizontalJerkWidget* max_hor_jerk_;
  field::MaxVerticalJerkWidget* max_ver_jerk_;
  field::AcceptanceRadiusWidget* acceptance_radius_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
};
}  // namespace ctrl
}  // namespace gui
