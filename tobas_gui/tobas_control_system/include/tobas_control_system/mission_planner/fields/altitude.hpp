// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::ctrl::field
{
class AltitudeWidget : public FieldWidget<double>
{
public:
  explicit AltitudeWidget();

  const char* label() const override;

  double getValue() const override;
  void setValue(double _value) override;

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace tobas::gui::ctrl::field
