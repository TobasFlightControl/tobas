#pragma once

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class AerodynamicsWidget_Manual : public AerodynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget_Manual();

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;
  double rotorDragCoef() const override;

private:
  ParamGetterWidget_DoubleSpinBox* motor_const_;
  ParamGetterWidget_DoubleSpinBox* moment_const_;
  ParamGetterWidget_DoubleSpinBox* rotor_drag_coef_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
