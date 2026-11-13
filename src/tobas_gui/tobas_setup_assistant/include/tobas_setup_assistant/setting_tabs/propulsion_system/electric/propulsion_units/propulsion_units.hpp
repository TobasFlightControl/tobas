// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./propulsion_unit.hpp"

namespace tobas::gui::sa::propulsion::electric
{
class PropulsionUnitsWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PropulsionUnitsWidget;
  using super = qt::TabWidget;

public:
  explicit PropulsionUnitsWidget(const uadf::Model& _uadf);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  int numUnits() const;

  QString linkName(int _index) const;

  /** Return the tab index, or -1 if it does not exist. */
  int index(const QString& _link_name) const;

  PropulsionUnitWidget* widget(int _index);
  const PropulsionUnitWidget* widget(int _index) const;
  PropulsionUnitWidget* widget(const QString& _link_name);
  const PropulsionUnitWidget* widget(const QString& _link_name) const;

private:
  const uadf::Model& uadf_;

private Q_SLOTS:
  void onCopyToAllButtonClicked(const QString& _link_name);
};
}  // namespace tobas::gui::sa::propulsion::electric
