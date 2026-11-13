// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/double_spin_box.hpp>

namespace tobas::qt
{
class LabeledDoubleSpinBox : public QWidget
{
  Q_OBJECT

  using self = LabeledDoubleSpinBox;

Q_SIGNALS:
  void valueChanged(double _value);

public:
  explicit LabeledDoubleSpinBox(const QString& _label_text);

  double getValue() const;
  bool setValue(double _value);

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged(double _value);

private:
  qt::DoubleSpinBox* data_;
};
}  // namespace tobas::qt
