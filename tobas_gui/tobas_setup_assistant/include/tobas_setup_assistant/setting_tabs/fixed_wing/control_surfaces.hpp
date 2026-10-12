// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/table_widget.hpp>
#include <tobas_uadf/model.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace fw
{
class ControlSurfacesWidget : public qt::TableWidget
{
  Q_OBJECT

  using super = qt::TableWidget;

public:
  explicit ControlSurfacesWidget(const uadf::Model& _uadf);

  void updateInternalDataStructures();
  void setToDefaults();
  bool isValid();

  YAML::Node dump() const;
  void load(const YAML::Node& _node);

  /** Number of registered control surfaces. */
  int numUnits() const;

  /** Return the currently selected link name, or an empty string if none exists. */
  QString selected() const;

  /** Return the table row for the link name, or -1 if it does not exist. */
  int find(const QString& _link_name) const;

  void add(const QString& _link_name);
  void remove(const QString& _link_name);

  // Getters
  QString linkName(int _row) const;
  QString jointName(int _row) const;
  double liftCoef(int _row) const;
  double dragCoef(int _row) const;
  double sideCoef(int _row) const;
  double rollCoef(int _row) const;
  double pitchCoef(int _row) const;
  double yawCoef(int _row) const;

  // Setters
  void linkName(int _row, const QString& _text);
  void jointName(int _row, const QString& _text);
  void liftCoef(int _row, double _value);
  void dragCoef(int _row, double _value);
  void sideCoef(int _row, double _value);
  void rollCoef(int _row, double _value);
  void pitchCoef(int _row, double _value);
  void yawCoef(int _row, double _value);

private:
  const uadf::Model& uadf_;

  void setToDefault(int _row);
};
}  // namespace fw
}  // namespace sa
}  // namespace gui
}  // namespace tobas
