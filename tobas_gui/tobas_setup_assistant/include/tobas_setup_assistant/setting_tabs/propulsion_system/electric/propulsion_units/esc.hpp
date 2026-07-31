// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/spin_box.hpp"

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
class ESCWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit ESCWidget();

  const char* name() const override;
  void setToDefaults() override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double maxCurrent() const;

private:
  ParamGetterWidget_SpinBox* max_current_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
