#pragma once

#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class EngineDynamicsWidget_Manual : public EngineDynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit EngineDynamicsWidget_Manual();

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double torqueConstant() const override;
  double dynamicFrictionTorque() const override;

private:
  ParamGetterWidget_DoubleSpinBox* torque_const_;
  ParamGetterWidget_DoubleSpinBox* friction_torque_;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
