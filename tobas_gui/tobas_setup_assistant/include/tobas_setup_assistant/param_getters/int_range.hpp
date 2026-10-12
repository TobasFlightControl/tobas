// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_std_tools/range.hpp>

#include "./base.hpp"
#include "./int_getter.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_IntRange : public ParamGetterWidget<st::Range<int>>
{
  Q_OBJECT

  using self = ParamGetterWidget_IntRange;
  using super = ParamGetterWidget<ValueType>;

Q_SIGNALS:
  void valueChanged(const ValueType& _value);

public:
  explicit ParamGetterWidget_IntRange(const QString& _param_name, const QString& _description_text = "");

  ValueType getValue() const override;
  bool setValue(const ValueType& _src) override;

  void setMinimum(int _minimum);
  void setMaximum(int _maximum);
  void setSingleStep(int _single_step);
  void setSuffix(const QString& _suffix);

  int min() const;
  int max() const;

  bool isValid() const;

private Q_SLOTS:
  void onValueChanged(int _value);

private:
  IntGetter* min_;
  IntGetter* max_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
