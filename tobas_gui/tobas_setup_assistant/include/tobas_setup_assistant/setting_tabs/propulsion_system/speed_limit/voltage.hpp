#pragma once

#include "./base.hpp"
#include "../motor.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
class SpeedLimitWidget_Voltage : public SpeedLimitWidget_Base
{
  Q_OBJECT

public:
  explicit SpeedLimitWidget_Voltage(MotorWidget* motor, AerodynamicsWidget* aerodynamics);

  const char* name() const override;

  void onInit() override;

  bool isValid() override;

  double maxRotSpeed() const override;

private:
  MotorWidget* motor_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
