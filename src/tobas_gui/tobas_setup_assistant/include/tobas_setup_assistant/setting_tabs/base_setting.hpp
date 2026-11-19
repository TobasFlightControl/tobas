// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

namespace tobas::gui::sa
{
class BaseSettingWidget : public QWidget
{
  Q_OBJECT

public:
  virtual const char* name() const = 0;
  virtual const char* title() const = 0;
  virtual const char* description() const = 0;

  /** Update the internal state according to URDF changes. */
  virtual void updateInternalDataStructures() = 0;

  /** Return all settings to their default values. */
  virtual void setToDefaults() = 0;

  /** Return true when user settings have no problems. */
  virtual bool isValid() = 0;

  /** Write user settings. */
  virtual YAML::Node dump() const = 0;

  /** Load user settings. */
  virtual void load(const YAML::Node& _node) = 0;
};
}  // namespace tobas::gui::sa
