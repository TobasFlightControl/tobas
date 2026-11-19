// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::sa::propulsion::electric
{
class BatteryWidget_LiPo : public BatteryWidget_Base
{
  Q_OBJECT

public:
  explicit BatteryWidget_LiPo();

  const char* name() const override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  double nominalVoltage() override;
  double maxVoltage() override;
  double sagVoltage() override;
  double maxCurrent() override;
  double capacity() override;
  double internalRegistance() override;

private:
  qt::SpinBox* num_cells_;
  qt::SpinBox* capacity_;
  qt::SpinBox* c_cont_;
  qt::SpinBox* registance_;
};
}  // namespace tobas::gui::sa::propulsion::electric
