// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_SpinBox : public ParamGetterWidget<int>
{
  Q_OBJECT

  using self = ParamGetterWidget_SpinBox;
  using super = ParamGetterWidget<int>;

Q_SIGNALS:
  void valueChanged(int _value);

public:
  explicit ParamGetterWidget_SpinBox(const QString& _param_name, const QString& _description_text = "");

  int getValue() const override;
  bool setValue(int _src) override;

  void setMinimum(int _minimum);
  void setMaximum(int _maximum);
  void setSingleStep(int _single_step);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged(int _value);

private:
  qt::SpinBox* spin_box_;
};
}  // namespace tobas::gui::sa
