#pragma once

#include "tobas_setup_assistant/param_getters/spin_box.hpp"

#include "./base.hpp"

namespace gui
{
namespace setup_assistant
{
class BatteryWidget_LiPo : public BatteryWidget_Base
{
  Q_OBJECT

  using self = BatteryWidget_LiPo;
  using super = BatteryWidget_Base;

  static constexpr double kNominalVoltagePerCell = 3.7;  // 1セルあたりの定格電圧
  static constexpr double kMaxVoltagePerCell = 4.2;      // 1セルあたりの最大電圧
  static constexpr double kSagVoltagePerCell = 3.4;      // 放電特性が急激に変化する電圧
  static constexpr double kVoltageThreshPerCell = 3.2;   // 内部抵抗による降圧を考慮した警告の閾値

public:
  explicit BatteryWidget_LiPo();

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
  ParamGetterWidget_SpinBox* num_cells_;
  ParamGetterWidget_SpinBox* capacity_;
  ParamGetterWidget_SpinBox* C_cont_;
  ParamGetterWidget_SpinBox* registance_;
};
};  // namespace setup_assistant
}  // namespace gui
