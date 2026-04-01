// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/labeled_double_spin_box.hpp>
#include <tobas_std_tools/range.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_DoubleRange : public ParamGetterWidget<st::Range<double>>
{
  Q_OBJECT

  using self = ParamGetterWidget_DoubleRange;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& value);

public:
  explicit ParamGetterWidget_DoubleRange(const QString& param_name, const QString& description_text = "");

  ValueType getValue() const override;
  bool setValue(const ValueType& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

  double min() const;
  double max() const;

  bool isValid() const;

private Q_SLOTS:
  void onValueChanged(double value);

private:
  qt::LabeledDoubleSpinBox* min_;
  qt::LabeledDoubleSpinBox* max_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
