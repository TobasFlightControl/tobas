// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_Ratio : public ParamGetterWidget<std::pair<int, int>>
{
  Q_OBJECT

  using self = ParamGetterWidget_Ratio;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& _value);

public:
  explicit ParamGetterWidget_Ratio(const QString& _param_name, const QString& _description_text = "");

  ValueType getValue() const override;
  bool setValue(const ValueType& _src) override;

  void setMinimum(int _minimum);
  void setMaximum(int _maximum);

  void setLeftText(const QString& _text);
  void setRightText(const QString& _text);

private Q_SLOTS:
  void onValueChanged();

private:
  qt::SpinBox* left_value_;
  qt::SpinBox* right_value_;
  QLabel* left_text_;
  QLabel* right_text_;
};
}  // namespace tobas::gui::sa
