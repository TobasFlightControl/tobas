// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class FixedWingWidget : public BaseControllerWidget
{
  Q_OBJECT

public:
  explicit FixedWingWidget();

  FrameType frameType() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  RcCommand acrobatModeCommand() const override;
  RcCommand stabilizeModeCommand() const override;
  RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  bool isValid() override;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
