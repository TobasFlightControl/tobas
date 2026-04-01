// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/vector3d_edit_horizontal.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
class ParamGetterWidget_Vector3d : public ParamGetterWidget<Eigen::Vector3d>
{
  Q_OBJECT

  using self = ParamGetterWidget_Vector3d;
  using super = ParamGetterWidget<Eigen::Vector3d>;

Q_SIGNALS:
  void valueChanged(const Eigen::Vector3d& value);

public:
  explicit ParamGetterWidget_Vector3d(const QString& param_name, const QString& description_text = "");

  Eigen::Vector3d getValue() const override;
  bool setValue(const Eigen::Vector3d& src) override;

  void setDecimals(int decimals);
  void setMinimum(double minimum);
  void setMaximum(double maximum);
  void setSingleStep(double single_step);
  void setSuffix(const QString& suffix);

private Q_SLOTS:
  void onValueChanged(const Eigen::Vector3d& value);

private:
  qt::Vector3dEditHorizontal* vector3d_;
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
