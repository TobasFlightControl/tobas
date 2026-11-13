// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>

#include "tobas_setup_assistant/param_getters/double_table.hpp"

#include "./base.hpp"
#include "./propeller.hpp"

namespace tobas::gui::sa::propulsion::ice
{
class AerodynamicsWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget(const PropellerWidget* _propeller);

  void setToDefaults() override;
  const char* name() const override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* _src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  VppMotorConstant motorConst() const;
  VppMomentConstant momentConst() const;
  VppDragConstant dragConst() const;

private:
  const PropellerWidget* const propeller_;

  ParamGetterWidget_DoubleTable* data_;

  std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> getData() const;
};
}  // namespace tobas::gui::sa::propulsion::ice
