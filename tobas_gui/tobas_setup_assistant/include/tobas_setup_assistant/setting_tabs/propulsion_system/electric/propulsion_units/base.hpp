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
class BaseSelectedLinkSettingWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual bool isValid() = 0;
  virtual void copyFrom(const BaseSelectedLinkSettingWidget* src) = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
