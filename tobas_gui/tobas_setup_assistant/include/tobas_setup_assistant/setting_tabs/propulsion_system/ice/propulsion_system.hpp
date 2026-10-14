// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../base.hpp"
#include "./engine/engine.hpp"
#include "./propulsion_units/propulsion_units.hpp"

namespace tobas::gui::sa::propulsion::ice
{
class PropulsionSystemWidget : public BasePropulsionSystemWidget
{
  Q_OBJECT

  using self = PropulsionSystemWidget;

public:
  EngineWidget* engine;
  PropulsionUnitsWidget* units;

  explicit PropulsionSystemWidget(const uadf::Model& _uadf);

  const char* name() const override;

  void updateInternalDataStructures() override;
  void setToDefaults() override;
  bool isValid() override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  PropulsionSystem type() const override;
  int numUnits() const override;

  QString linkName(int _index) const override;
};
}  // namespace tobas::gui::sa::propulsion::ice
