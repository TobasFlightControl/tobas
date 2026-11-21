// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

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
class BatteryWidget_Other : public BatteryWidget_Base
{
  Q_OBJECT

  using self = BatteryWidget_Other;
  using super = BatteryWidget_Base;

public:
  explicit BatteryWidget_Other();

  const char* name() const override;

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
  qt::DoubleSpinBox* nominal_voltage_;
  qt::DoubleSpinBox* max_voltage_;
  qt::DoubleSpinBox* sag_voltage_;
  qt::SpinBox* max_current_;
  qt::SpinBox* capacity_;
  qt::SpinBox* registance_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
