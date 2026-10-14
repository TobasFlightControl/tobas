// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_DoublePair : public ParamGetterWidget<std::pair<double, double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoublePair;
  using super = ParamGetterWidget<std::pair<double, double>>;

Q_SIGNALS:
  void valueChanged(std::pair<double, double> _value);

public:
  explicit ParamGetterWidget_DoublePair(const QString& _param_name, const QString& _description_text = "");

  std::pair<double, double> getValue() const override;
  bool setValue(const std::pair<double, double>& _src) override;

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

  double first() const;
  double second() const;

private Q_SLOTS:
  void onValueChanged(double _value);

private:
  qt::DoubleSpinBox* first_;
  qt::DoubleSpinBox* second_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
