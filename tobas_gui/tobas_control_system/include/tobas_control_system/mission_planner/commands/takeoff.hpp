#pragma once

#include "../fields/altitude.hpp"
#include "../fields/altitude_frame.hpp"
#include "../fields/altitude_tolerance.hpp"
#include "../fields/takeoff_max_accel.hpp"
#include "../fields/takeoff_max_jerk.hpp"
#include "../fields/takeoff_max_speed.hpp"
#include "./base.hpp"

namespace gui
{
namespace ctrl
{
struct TakeoffData : public BaseCommandData
{
  using SharedPtr = std::shared_ptr<TakeoffData>;

  double altitude;  // [m]
  AltitudeFrame altitude_frame;
  double max_speed;           // [m/s]
  double max_accel;           // [m/s^2]
  double max_jerk;            // [m/s^3]
  double altitude_tolerance;  // [m]

  Command type() const override
  {
    return Command::kTakeoff;
  }
};

class TakeoffWidget : public BaseCommandWidget
{
  Q_OBJECT

  using self = TakeoffWidget;
  using super = BaseCommandWidget;

public:
  explicit TakeoffWidget();

  const char* name() const override;
  BaseCommandData::SharedPtr data() const override;

  double altitude() const;
  AltitudeFrame altitudeFrame() const;
  double maxSpeed() const;
  double maxAccel() const;
  double maxJerk() const;
  double altitudeTolerance() const;

  void altitude(double value);
  void altitudeFrame(AltitudeFrame value);
  void maxSpeed(double value);
  void maxAccel(double value);
  void maxJerk(double value);
  void altitudeTolerance(double value);

private:
  field::AltitudeWidget* altitude_;
  field::AltitudeFrameWidget* altitude_frame_;
  field::TakeoffMaxSpeedWidget* max_speed_;
  field::TakeoffMaxAccelWidget* max_accel_;
  field::TakeoffMaxJerkWidget* max_jerk_;
  field::AltitudeToleranceWidget* altitude_tolerance_;
};
}  // namespace ctrl
}  // namespace gui
