// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./base.hpp"
#include "tobas_setup_assistant/param_getters/double_table.hpp"

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
/**
 * @brief Obtain thrust and torque coefficients from Thrust Stand experimental data.
 * Obtain the drag coefficient from Blade Theory.
 */
class AerodynamicsWidget_ThrustStand : public AerodynamicsWidget_Base
{
  Q_OBJECT

public:
  explicit AerodynamicsWidget_ThrustStand(rclcpp::Node::SharedPtr node);

  const char* name() const override;
  const char* description() const override;

  bool isValid() override;
  void copyFrom(const AerodynamicsWidget_Base* src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& node) override;

  double motorConst() const override;
  double momentConst() const override;

private:
  ParamGetterWidget_DoubleTable* data_;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
}  // namespace tobas
