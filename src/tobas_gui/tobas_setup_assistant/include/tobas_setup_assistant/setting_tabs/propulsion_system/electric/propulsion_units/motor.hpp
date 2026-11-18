// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::sa::propulsion::electric
{
class MotorWidget : public BaseSelectedLinkSettingWidget
{
  Q_OBJECT

public:
  explicit MotorWidget();

  const char* name() const override;
  void setToDefaults() override;
  bool isValid() override;
  void copyFrom(const BaseSelectedLinkSettingWidget* _src) override;

  YAML::Node dump() const override;
  void load(const YAML::Node& _node) override;

  /** Number of poles [-] */
  int numPoles() const;

  /** Kv [rad/s/V] */
  double kv() const;

  /** Internal resistance [Ω] */
  double internalResistance() const;

  /** Minimum rotation speed [rad/s] */
  double minimumSpeed() const;

private:
  qt::SpinBox* num_poles_;
  qt::SpinBox* kv_;
  qt::SpinBox* resistance_;
  qt::SpinBox* min_speed_;
};
}  // namespace tobas::gui::sa::propulsion::electric
