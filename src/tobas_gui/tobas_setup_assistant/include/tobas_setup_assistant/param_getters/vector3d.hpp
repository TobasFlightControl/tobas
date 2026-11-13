// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp>

#include "./base.hpp"

namespace tobas::gui::sa
{
class ParamGetterWidget_Vector3d : public ParamGetterWidget<Eigen::Vector3d>
{
  Q_OBJECT

  using self = ParamGetterWidget_Vector3d;
  using super = ParamGetterWidget<Eigen::Vector3d>;

Q_SIGNALS:
  void valueChanged(const Eigen::Vector3d& _value);

public:
  explicit ParamGetterWidget_Vector3d(const QString& _param_name, const QString& _description_text = "");

  Eigen::Vector3d getValue() const override;
  bool setValue(const Eigen::Vector3d& _src) override;

  void setDecimals(int _decimals);
  void setMinimum(double _minimum);
  void setMaximum(double _maximum);
  void setSingleStep(double _single_step);
  void setSuffix(const QString& _suffix);

private Q_SLOTS:
  void onValueChanged(const Eigen::Vector3d& _value);

private:
  qt::Vector3dEditHorizontal* vector3d_;
};
}  // namespace tobas::gui::sa
