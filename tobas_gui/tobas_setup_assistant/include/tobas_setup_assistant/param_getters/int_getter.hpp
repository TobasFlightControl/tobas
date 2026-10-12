// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/spin_box.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
class IntGetter : public QWidget
{
  Q_OBJECT

  using self = IntGetter;

Q_SIGNALS:
  void valueChanged(int _value);

public:
  explicit IntGetter(const QString& _name);

  int getValue() const;
  bool setValue(int _value);

  void setMinimum(int _minimum);
  void setMaximum(int _maximum);
  void setSingleStep(int _single_step);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged(int _value);

private:
  qt::SpinBox* data_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
