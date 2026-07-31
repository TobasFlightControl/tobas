// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>
#include <QWidget>

#include <tobas_constants/rc_command.hpp>

#include "tobas_setup_assistant/frame_type.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class BaseControllerWidget : public QWidget
{
  Q_OBJECT

public:
  virtual void setToDefaults() = 0;

  virtual FrameType frameType() const = 0;
  virtual QString controllerPackage() const = 0;
  virtual QString pluginName() const = 0;

  virtual RcCommand acrobatModeCommand() const = 0;
  virtual RcCommand stabilizeModeCommand() const = 0;
  virtual RcCommand loiterModeCommand() const = 0;

  /* Static private ROS parameters. */
  virtual YAML::Node staticParams() const = 0;

  virtual YAML::Node dump() const = 0;
  virtual void load(const YAML::Node& node) = 0;

  /* Return true when user settings are valid. */
  virtual bool isValid() = 0;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
