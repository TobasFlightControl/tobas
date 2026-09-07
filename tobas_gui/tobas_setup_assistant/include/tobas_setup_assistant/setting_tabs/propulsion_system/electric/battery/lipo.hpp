// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/spin_box.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
class BatteryWidget_LiPo : public BatteryWidget_Base
{
  Q_OBJECT

  using self = BatteryWidget_LiPo;
  using super = BatteryWidget_Base;

public:
  explicit BatteryWidget_LiPo();

  const char* name() const override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
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
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
