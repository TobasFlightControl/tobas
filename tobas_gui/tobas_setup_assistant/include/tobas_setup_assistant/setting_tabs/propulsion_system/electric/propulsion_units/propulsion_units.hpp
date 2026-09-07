// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/tab_widget.hpp>
#include <tobas_uadf/model.hpp>

#include "./propulsion_unit.hpp"

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
class PropulsionUnitsWidget : public qt::TabWidget
{
  Q_OBJECT

  using self = PropulsionUnitsWidget;
  using super = qt::TabWidget;

public:
  explicit PropulsionUnitsWidget(const uadf::Model& uadf);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& node);

  int numUnits() const;

  QString linkName(int index) const;

  /* Return the tab index, or -1 if it does not exist. */
  int index(const QString& link_name) const;

  PropulsionUnitWidget* widget(int index);
  const PropulsionUnitWidget* widget(int index) const;
  PropulsionUnitWidget* widget(const QString& link_name);
  const PropulsionUnitWidget* widget(const QString& link_name) const;

private:
  const uadf::Model& uadf_;

private Q_SLOTS:
  void onCopyToAllButtonClicked(const QString& link_name);
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
