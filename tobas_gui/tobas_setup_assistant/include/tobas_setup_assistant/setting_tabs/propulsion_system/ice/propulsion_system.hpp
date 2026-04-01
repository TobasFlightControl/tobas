// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../base.hpp"
#include "./engine/engine.hpp"
#include "./propulsion_units/propulsion_units.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
class PropulsionSystemWidget : public BasePropulsionSystemWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;
  using super = BasePropulsionSystemWidget;

  static constexpr char kEngineTitle[] = "Engine";
  static constexpr char kPropulsionUnitTitle[] = "Propulsion Units";

public:
  EngineWidget* engine;
  PropulsionUnitsWidget* units;

  explicit PropulsionSystemWidget(rclcpp::Node::SharedPtr node, const uadf::Model& uadf);

  const char* name() const override;

  void updateInternalDataStructures() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  PropulsionSystem type() const override;
  int numUnits() const override;

  QString linkName(int index) const override;
};
};  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
