// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_qt_tools/widgets/labeled_double_spin_box.hpp>
#include <tobas_std_tools/range.hpp>

namespace tobas::gui::sa
{
class DoubleRangeWidget : public QWidget
{
  Q_OBJECT

  using self = DoubleRangeWidget;
  using super = QWidget;

Q_SIGNALS:
  void valueChanged(const st::Range<double>& _value);

public:
  explicit DoubleRangeWidget(QWidget* _parent = nullptr);

  st::Range<double> value() const;
  void setValue(const st::Range<double>& _value);

  bool isValid() const;

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setRange(double _minimum, double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

private:
  qt::LabeledDoubleSpinBox* min_;
  qt::LabeledDoubleSpinBox* max_;

private Q_SLOTS:
  void onValueChanged();
};
}  // namespace tobas::gui::sa
