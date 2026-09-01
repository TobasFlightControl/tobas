// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/combo_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace ctrl
{
class CustomFrameWidget : public BaseControllerWidget
{
  Q_OBJECT

public:
  explicit CustomFrameWidget();

  void setToDefaults() override;

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

private:
  qt::ComboBox* acrobat_mode_;
  qt::ComboBox* stabilize_mode_;
  qt::ComboBox* loiter_mode_;
};
}  // namespace ctrl
}  // namespace sa
}  // namespace gui
}  // namespace tobas
