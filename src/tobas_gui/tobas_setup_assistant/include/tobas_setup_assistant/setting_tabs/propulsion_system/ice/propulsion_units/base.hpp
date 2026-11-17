// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_qt_tools/widgets/scroll_area.hpp>

namespace tobas::gui::sa::propulsion::ice
{
class BaseSelectedLinkSettingWidget : public qt::ScrollArea
{
  Q_OBJECT

public:
  explicit BaseSelectedLinkSettingWidget();

  virtual const char* name() const = 0;
  virtual void setToDefaults() = 0;
  virtual bool isValid() = 0;
  virtual void copyFrom(const BaseSelectedLinkSettingWidget* _src) = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& _node) = 0;
};
}  // namespace tobas::gui::sa::propulsion::ice
