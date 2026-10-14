// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"

namespace tobas::gui::sa::ctrl
{
class FixedWingWidget : public BaseControllerWidget
{
  Q_OBJECT

public:
  explicit FixedWingWidget();

  void setToDefaults() override;

  FrameType frameType() const override;
  QString controllerPackage() const override;
  QString pluginName() const override;

  RcCommand acrobatModeCommand() const override;
  RcCommand stabilizeModeCommand() const override;
  RcCommand loiterModeCommand() const override;

  YAML::Node staticParams() const override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  bool isValid() override;
};
}  // namespace tobas::gui::sa::ctrl
