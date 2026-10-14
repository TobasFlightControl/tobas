// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/ratio.hpp"

namespace tobas::gui::sa::propulsion::ice
{
class TransmissionWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit TransmissionWidget();

  const char* name() const override;
  void setToDefaults() override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* _src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  double gearRatio() const;

private:
  ParamGetterWidget_Ratio* gear_ratio_;
};
}  // namespace tobas::gui::sa::propulsion::ice
