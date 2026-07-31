// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_qt_tools/widgets/stacked_widget.hpp>

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
class BatteryWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = BatteryWidget;
  using super = qt::ScrollArea;

  static constexpr char kTypeKey[] = "battery_type";

public:
  explicit BatteryWidget();

  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  double nominalVoltage();
  double maxVoltage();
  double sagVoltage();
  double maxCurrent();
  double capacity();
  double internalRegistance();

private:
  qt::ComboBox* type_;
  qt::StackedWidget* batteries_;

  BatteryWidget_Base* widget(int index);
  const BatteryWidget_Base* widget(int index) const;

  BatteryWidget_Base* selected();
  const BatteryWidget_Base* selected() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
