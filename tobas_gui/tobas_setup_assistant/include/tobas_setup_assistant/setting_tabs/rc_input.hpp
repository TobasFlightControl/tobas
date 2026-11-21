// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>
#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base_setting.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class RcInputWidget : public BaseSettingWidget
{
  Q_OBJECT

  using self = RcInputWidget;
  using super = BaseSettingWidget;

public:
  explicit RcInputWidget();

  const char* name() const override;
  const char* title() const override;
  const char* description() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  int numOfSbusChannels() const;
  double armDuration() const;
  double disarmDuration() const;

private:
  qt::SpinBox* num_sbus_channels_;
  qt::DoubleSpinBox* arm_duration_;
  qt::DoubleSpinBox* disarm_duration_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
