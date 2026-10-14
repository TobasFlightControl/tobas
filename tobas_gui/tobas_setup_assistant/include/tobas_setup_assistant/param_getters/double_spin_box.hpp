// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_DoubleSpinBox : public ParamGetterWidget<double>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleSpinBox;
  using super = ParamGetterWidget<double>;

Q_SIGNALS:
  void valueChanged(double _value);

public:
  explicit ParamGetterWidget_DoubleSpinBox(const QString& _param_name, const QString& _description_text = "");

  double getValue() const override;
  bool setValue(double _src) override;

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged(double _value);

private:
  qt::DoubleSpinBox* spin_box_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
