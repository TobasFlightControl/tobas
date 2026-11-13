// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_LinearEquation : public ParamGetterWidget<std::pair<double, double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_LinearEquation;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& _value);

public:
  explicit ParamGetterWidget_LinearEquation(
    const QString& _param_name,
    const QString& _left,
    const QString& _value,
    const QString& _description_text = "");

  ValueType getValue() const override;
  bool setValue(const ValueType& _src) override;

  void setDecimals(int _decimals);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged();

private:
  qt::DoubleSpinBox* c0_;
  qt::DoubleSpinBox* c1_;
  QLabel* suffix_;
};
}  // namespace tobas::gui::sa
