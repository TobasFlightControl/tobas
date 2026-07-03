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
namespace mission
{
class BaseExecutorWidget : public QWidget
{
  Q_OBJECT

public:
  virtual QString executorPackage() const = 0;
  virtual QString pluginName() const = 0;

  /* Static private ROS parameters. */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* Return true when user settings are valid. */
  virtual bool isValid() = 0;
};
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
