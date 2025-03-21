#pragma once

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "tobas_setup_assistant/param_getters/linear_equation.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
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

  std::pair<double, double> motorConst() const override;
  double momentConst() const override;
  std::pair<double, double> dragConst() const override;

private:
  ParamGetterWidget_LinearEquation* motor_const_;
  ParamGetterWidget_DoubleSpinBox* moment_const_;
  ParamGetterWidget_LinearEquation* drag_const_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
