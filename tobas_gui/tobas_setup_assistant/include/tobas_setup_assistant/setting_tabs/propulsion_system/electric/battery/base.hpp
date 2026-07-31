// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

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
class BatteryWidget_Base : public QWidget
{
  Q_OBJECT

  using self = BatteryWidget_Base;
  using super = QWidget;

public:
  virtual const char* name() const = 0;
  virtual void setToDefaults() = 0;
  virtual bool isValid() = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* Nominal voltage [V]. */
  virtual double nominalVoltage() = 0;

  /* Maximum voltage [V]. */
  virtual double maxVoltage() = 0;

  /* Voltage where discharge characteristics change abruptly [V]. */
  virtual double sagVoltage() = 0;

  /* Maximum continuous current [A]. */
  virtual double maxCurrent() = 0;

  /* Electric capacity [As]. */
  virtual double capacity() = 0;

  /* Internal resistance [ohm]. */
  virtual double internalRegistance() = 0;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
