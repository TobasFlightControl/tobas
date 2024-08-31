#pragma once

#include "./base.hpp"
#include "../motor.hpp"
#include "../aerodynamics/aerodynamics.hpp"

namespace gui
{
namespace setup_assistant
{
class ElectrodynamicsWidget_Spec : public ElectrodynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit ElectrodynamicsWidget_Spec(MotorWidget* motor, AerodynamicsWidget* aerodynamics);

  const char* name() const override;
  const char* description() const override;

  void onInit() override;

  bool isValid() override;
  void copyFrom(const ElectrodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  std::pair<double, double> rotSpeedCoefs() const override;

private:
  MotorWidget* motor_;
  AerodynamicsWidget* aerodynamics_;
};
}  // namespace setup_assistant
}  // namespace gui
