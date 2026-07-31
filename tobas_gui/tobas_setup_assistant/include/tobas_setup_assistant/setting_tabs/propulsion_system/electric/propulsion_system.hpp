// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../base.hpp"
#include "./battery/battery.hpp"
#include "./propulsion_units/propulsion_units.hpp"

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
class PropulsionSystemWidget : public BasePropulsionSystemWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BasePropulsionSystemWidget;

  static constexpr char kBatteryTitle[] = "Battery";
  static constexpr char kPropulsionUnitTitle[] = "Propulsion Units";

public:
  BatteryWidget* battery;
  PropulsionUnitsWidget* units;

  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf);

  const char* name() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  PropulsionSystem type() const override;
  int numUnits() const override;

  QString linkName(int index) const override;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
