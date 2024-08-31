#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"
#include "tobas_setup_assistant/param_getters/double_spin_box.hpp"
#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class BatteryWidget_Other : public BatteryWidget_Base
{
  Q_OBJECT

  using self = BatteryWidget_Other;
  using super = BatteryWidget_Base;

public:
  explicit BatteryWidget_Other();

  const char* name() const override;

  bool isValid() override;

  YAML::Node dump() override;
  void load(const YAML::Node& node) override;

  double nominalVoltage() override;
  double maxVoltage() override;
  double sagVoltage() override;
  double maxCurrent() override;
  double capacity() override;
  double internalRegistance() override;

private:
  ParamGetterWidget_DoubleSpinBox* nominal_voltage_;
  ParamGetterWidget_DoubleSpinBox* max_voltage_;
  ParamGetterWidget_DoubleSpinBox* sag_voltage_;
  ParamGetterWidget_DoubleSpinBox* max_current_;
  ParamGetterWidget_SpinBox* capacity_;
  ParamGetterWidget_SpinBox* registance_;
};
};  // namespace setup_assistant
}  // namespace gui
